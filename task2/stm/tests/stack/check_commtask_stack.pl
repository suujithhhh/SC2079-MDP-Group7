#!/usr/bin/perl
# CommTask static stack-budget gate (host tool; needs perl + arm-none-eabi-objdump).
#
#   perl tests/stack/check_commtask_stack.pl <RTOSV.elf | disassembly.txt> [--census] [-v]
#                                            [--main Core/Src/main.c] [--ioc RTOSV.ioc]
#
# Build the firmware out-of-tree first (never in the tracked Debug/ directory), e.g. with the
# project's own flags, then point this at the resulting ELF. It disassembles the LINKED image,
# derives every function's frame from its prologue, builds the call graph from StartCommTask and
# reports the worst-case stack depth of EVERY reachable chain against CommTask's 2048-byte stack.
#
# WHAT IT CHECKS
#   Allocation: CommTask's stack is read from BOTH Core/Src/main.c (CommTask_attributes
#               .stack_size = N * 4) and RTOSV.ioc (FREERTOS.Tasks01 ... CommTask,24,<words>,...).
#               They must agree and be exactly 2048 B (512 words). 1024 B (the original value) and
#               1536 B (only ~28 B of margin after the reserve) are rejected explicitly.
#   Budget    : every chain must satisfy  depth + RESERVE + MIN_MARGIN <= STACK,
#               i.e. depth <= 2048 - 204 - 100 = 1744 B. There is no "known over budget" class:
#               a single chain over budget fails the run.
#               - the six USPARK/STOP/telemetry/MOV chains are listed individually;
#               - the census checks EVERY function reachable from StartCommTask (its worst chain =
#                 longest caller-frame prefix + its own deepest callee chain) and every parser
#                 callback reached through vision_rx_dispatch().
#   Structure : the properties that keep the STOP chains short (a runtime test cannot see stack
#               frames): exact STOP is dispatched from vision_rx_feed() BEFORE the large
#               vision_rx_dispatch() frame; the dispatcher does not reference on_stop or the
#               USPARK handlers; on_stop() has no wrapper frame; the STOP helper, telemetry
#               service and parking pass keep their large structures off the stack and are not
#               nested under one another.
#
# MODEL AND ITS LIMITS (read before trusting a number)
#   * Frame = bytes pushed (push/stmdb/vpush) + `sub sp,#imm` in the first 10 instructions.
#     Validated against GCC -fstack-usage: identical for application functions at -O0.
#   * Edges: bl / blx-immediate are calls; a direct b to another function's entry is a tail call
#     (does not add the caller's frame). Indirect calls are NOT followed unless listed in
#     EXTRA_EDGES; the audit below prints every reachable function that has one.
#   * RESERVE (204 B) = hardware exception frame with FPU (104 B) + FreeRTOS PendSV register
#     save with FPU (100 B): what a preempted task needs on top of its own depth. Interrupt
#     handlers themselves run on MSP and are not counted.
#   * Library policy (newlib-nano snprintf): the buffer-growth path
#     (__ssputs_r -> _malloc_r/_realloc_r/_free_r/sbrk) is EXCLUDED because it is unreachable for a
#     fixed-buffer snprintf: sniprintf() sets the fake FILE flags to 0x208 and the growth block is
#     entered only if (flags & 0x480) != 0 (tst.w r0,#0x480; beq in __ssputs_r). The printf
#     callback __ssputs_r is passed to _printf_i as a function pointer (literal-pool word in
#     _svfiprintf_r) and called with blx r9, so EXTRA_EDGES adds it. Nothing else is excluded.
#   * Paths through abort()/raise()/fflush at exit are termination paths, not operation.
#   * It is a static worst case over all code paths, not a measurement. The physical stack
#     high-water mark on the robot remains a mandatory bench gate.
use strict;
use warnings;
no warnings 'recursion';

my $EXPECTED_STACK = 2048;   # production CommTask allocation in bytes (512 words)
my $RESERVE    = 204;
my $MIN_MARGIN = 100;        # required slack beyond the reserve
my @EXCLUDE    = qw(_malloc_r _realloc_r _free_r);
my @EXTRA_EDGES = (['_printf_i', '__ssputs_r'], ['_printf_common', '__ssputs_r']);

my ($input, $census, $verbose) = (undef, 0, 0);
my $root_dir = $0;
$root_dir = ($root_dir =~ m{^(.*)[/\\]tests[/\\]stack[/\\][^/\\]*$}) ? $1 : '.';
my $main_path = "$root_dir/Core/Src/main.c";
my $ioc_path  = "$root_dir/RTOSV.ioc";
while (@ARGV) {
    my $a = shift @ARGV;
    if    ($a eq '--census') { $census = 1 }
    elsif ($a eq '-v')       { $verbose = 1 }
    elsif ($a eq '--main')   { $main_path = shift @ARGV }
    elsif ($a eq '--ioc')    { $ioc_path = shift @ARGV }
    elsif (!defined $input)  { $input = $a }
}
die "usage: $0 <RTOSV.elf|disassembly.txt> [--census] [-v] [--main main.c] [--ioc RTOSV.ioc]\n" unless defined $input;

my $dis = $input;
my $tmp;
if ($input =~ /\.elf$/i) {
    my $objdump = $ENV{OBJDUMP} || 'arm-none-eabi-objdump';
    $tmp = "$input.dis.tmp";
    system("\"$objdump\" -d \"$input\" > \"$tmp\"") == 0 or die "objdump failed ($objdump)\n";
    $dis = $tmp;
}

# ------------------------------------------------------------------ parse
my (%frame, %calls, %tails, %indirect);
{
    open(my $fh, '<', $dis) or die "$dis: $!";
    my ($cur, $n);
    while (my $line = <$fh>) {
        chomp $line;
        if ($line =~ /^[0-9a-f]{8} <([^>]+)>:\s*$/) { $cur = $1; $n = 0; $frame{$cur} = 0; next }
        next unless defined $cur;
        my @f = split /\t/, $line;
        next unless @f >= 3 && $f[0] =~ /^\s*[0-9a-f]+:\s*$/;
        my ($mn, $ops) = ($f[2], $f[3] // '');
        $mn =~ s/\s+$//; $ops =~ s/\s*@.*$//;
        $n++;
        if ($n <= 10) {
            if ($mn =~ /^(push|stmdb)/ && $ops =~ /\{([^}]*)\}/) {
                my @r = split /,/, $1;
                $frame{$cur} += 4 * scalar(@r) if $mn eq 'push' || $ops =~ /^sp!/;
            } elsif ($mn =~ /^vpush/ && $ops =~ /\{([^}]*)\}/) {
                my $c = 0;
                for my $r (split /,/, $1) { if ($r =~ /d(\d+)-d(\d+)/) { $c += $2 - $1 + 1 } else { $c++ } }
                $frame{$cur} += 8 * $c;
            } elsif ($mn =~ /^sub(w|\.w)?$/ && $ops =~ /^sp,\s*(?:sp,\s*)?#(\d+)/) {
                $frame{$cur} += $1;
            }
        }
        if    ($mn eq 'bl')   { $calls{$cur}{$1} = 1 if $ops =~ /<([^>+]+)>/ }
        elsif ($mn eq 'blx')  { if ($ops =~ /<([^>+]+)>/) { $calls{$cur}{$1} = 1 } else { $indirect{$cur}++ } }
        elsif ($mn =~ /^b(\.w|\.n)?$/ && $ops =~ /<([^>+]+)>/) { my $t = $1; $tails{$cur}{$t} = 1 if $t ne $cur }
        elsif ($mn eq 'bx' && $ops !~ /lr/) { $indirect{$cur}++ }
    }
    close($fh);
}
for my $e (@EXTRA_EDGES) { $calls{$e->[0]}{$e->[1]} = 1 }
my %excl = map { $_ => 1 } @EXCLUDE;

my (%memo, %busy);
sub depth {
    my ($f) = @_;
    return @{ $memo{$f} } if $memo{$f};
    return (0, [$f]) unless exists $frame{$f};
    return (0, [$f . ' (recursion)']) if $busy{$f};
    $busy{$f} = 1;
    my ($best, $bp) = (0, []);
    for my $c (keys %{ $calls{$f} || {} }) {
        next if !exists $frame{$c} || $excl{$c};
        my ($d, $p) = depth($c);
        ($best, $bp) = ($d, $p) if $d > $best;
    }
    my ($total, $path) = ($frame{$f} + $best, ["$f($frame{$f})", @$bp]);
    for my $c (keys %{ $tails{$f} || {} }) {
        next if !exists $frame{$c} || $excl{$c};
        my ($d, $p) = depth($c);
        ($total, $path) = ($d, ["$f(tail)", @$p]) if $d > $total;
    }
    $busy{$f} = 0;
    $memo{$f} = [$total, $path];
    return ($total, $path);
}
sub has_edge { my ($a, $b) = @_; return (($calls{$a} || {})->{$b} || ($tails{$a} || {})->{$b}) ? 1 : 0 }

# ------------------------------------------------------------------ required symbols
for my $s (qw(StartCommTask AppComm_ProcessByte vision_rx_feed vision_rx_dispatch
              vision_rx_dispatch_us_park vision_rx_dispatch_ir on_stop on_us_park_get on_us_park_set
              on_ir_get on_usd_get on_move vision_rx_dispatch_fir
              AppComm_ServiceTelemetry AppComm_ServiceParking AppComm_RequestPriorityStop)) {
    die "symbol $s not found: build with the Debug (-O0) configuration so static functions keep their names\n"
        unless exists $frame{$s};
}

my $fail = 0;
sub check { my ($what, $ok) = @_; printf "  %-4s %s\n", ($ok ? 'ok' : 'FAIL'), $what; $fail++ unless $ok }

# ------------------------------------------------------------------ allocation
sub slurp { my ($p) = @_; open(my $fh, '<', $p) or return undef; binmode($fh); local $/; my $t = <$fh>; close($fh); return $t }
my ($main_stack, $ioc_stack);
{
    my $m = slurp($main_path);
    $main_stack = 4 * $1 if defined $m && $m =~ /CommTask_attributes\s*=\s*\{[^}]*?\.stack_size\s*=\s*(\d+)\s*\*\s*4\s*,/s;
    my $i = slurp($ioc_path);
    $ioc_stack  = 4 * $1 if defined $i && $i =~ /^FREERTOS\.Tasks01=[^\r\n]*?(?:=|;)CommTask,\d+,(\d+),StartCommTask,/m;
}
print "CommTask allocation:\n";
check("main.c  CommTask_attributes.stack_size readable ($main_path)" . (defined $main_stack ? ": $main_stack B" : ''), defined $main_stack);
check("RTOSV.ioc CommTask stack field readable ($ioc_path)"          . (defined $ioc_stack  ? ": $ioc_stack B"  : ''), defined $ioc_stack);
if (defined $main_stack && defined $ioc_stack) {
    check("RTOSV.ioc ($ioc_stack B) and main.c ($main_stack B) agree", $main_stack == $ioc_stack);
}
for my $v ([$main_stack, 'main.c'], [$ioc_stack, 'RTOSV.ioc']) {
    next unless defined $v->[0];
    check("$v->[1]: CommTask has not fallen back to 1024 B", $v->[0] != 1024);
    check("$v->[1]: CommTask is not the rejected 1536 B",    $v->[0] != 1536);
    check("$v->[1]: CommTask is exactly $EXPECTED_STACK B (is $v->[0] B)", $v->[0] == $EXPECTED_STACK);
}
my $STACK = $main_stack // $ioc_stack // $EXPECTED_STACK;
my $limit = $STACK - $RESERVE - $MIN_MARGIN;

# ------------------------------------------------------------------ reachability and entry depth
# entry{f} = largest stack in use by the callers' frames when f is entered (tail calls add none).
my (%seen, %onstack, @post, @recursive);
sub succ_of { my ($f) = @_; return grep { exists $frame{$_} && !$excl{$_} } (keys %{ $calls{$f} || {} }, keys %{ $tails{$f} || {} }) }
sub dfs {
    my ($f) = @_;
    return if $seen{$f};
    $seen{$f} = 1; $onstack{$f} = 1;
    for my $c (sort(succ_of($f))) {
        if ($onstack{$c}) { push @recursive, "$f -> $c"; next }
        dfs($c);
    }
    $onstack{$f} = 0;
    push @post, $f;
}
dfs('StartCommTask');
my %entry = map { $_ => 0 } @post;
for my $f (reverse @post) {
    for my $c (keys %{ $calls{$f} || {} }) {
        next if !exists $frame{$c} || $excl{$c} || !$seen{$c};
        my $e = $entry{$f} + $frame{$f};
        $entry{$c} = $e if $e > $entry{$c};
    }
    for my $c (keys %{ $tails{$f} || {} }) {
        next if !exists $frame{$c} || $excl{$c} || !$seen{$c};
        $entry{$c} = $entry{$f} if $entry{$f} > $entry{$c};
    }
}

# ------------------------------------------------------------------ budget: required chains
my $prefix_parser   = $frame{StartCommTask} + $frame{AppComm_ProcessByte} + $frame{vision_rx_feed};
my $prefix_dispatch = $prefix_parser + $frame{vision_rx_dispatch};
my $prefix_early    = $prefix_parser + $frame{vision_rx_dispatch_us_park};

# The prefix is derived from the call graph, not assumed: if the large dispatcher calls a handler
# (the pre-fast-path design) its frame is on the stack under that handler, whatever the structure
# checks below say.
my $stop_prefix = $prefix_parser + (has_edge('vision_rx_dispatch', 'on_stop') ? $frame{vision_rx_dispatch} : 0);
my $us_prefix   = has_edge('vision_rx_feed', 'vision_rx_dispatch_us_park')
                  ? $prefix_early
                  : $prefix_dispatch + $frame{vision_rx_dispatch_us_park};

# IR,GET (Phase 3): recognised in vision_rx_feed() before the large dispatcher, like USPARK. Its parser
# callback only counts a request; the reply is formatted by AppComm_TrySendIrGet() from the telemetry
# service (present only in a build with APP_IR_ENABLE).
my $ir_prefix = has_edge('vision_rx_feed', 'vision_rx_dispatch_ir')
                ? $prefix_parser + $frame{vision_rx_dispatch_ir}
                : $prefix_dispatch + $frame{vision_rx_dispatch_ir};
my $has_ir_reply = exists $frame{AppComm_TrySendIrGet};

# MOV,FIRL / MOV,FIRR (Phase 5): recognised in vision_rx_feed() before the dispatcher too (its own helper);
# the request goes through on_move() -> AppComm_QueueMotionCommand(). The IREDGE, decision line is formatted
# by AppComm_TrySendIrEdge() from the telemetry service (present only with APP_IR_ENABLE).
my $fir_prefix = has_edge('vision_rx_feed', 'vision_rx_dispatch_fir')
                 ? $prefix_parser + $frame{vision_rx_dispatch_fir}
                 : $prefix_dispatch + $frame{vision_rx_dispatch_fir};
my $has_edge_line = exists $frame{AppComm_TrySendIrEdge};

my @chains = (
    ['1  manual STOP (parser fast path)',  $stop_prefix,             'on_stop'],
    ['2  automatic USPARK STOP',           $frame{StartCommTask},    'AppComm_ServiceParking'],
    ['3a USPARK,GET (early path)',         $us_prefix,               'on_us_park_get'],
    ['3b USPARK,1 / USPARK,0 (early path)',$us_prefix,               'on_us_park_set'],
    ['4  ordinary MOV parsing',            $prefix_dispatch,         'on_move'],
    ['3c IR,GET request (early path)',     $ir_prefix,               'on_ir_get'],
    ['3e USD,GET request (early path)',    $ir_prefix,               'on_usd_get'],
    ['5  telemetry service',               $frame{StartCommTask},    'AppComm_ServiceTelemetry'],
);
push @chains, ['6  IR,GET reply (telemetry)', $frame{StartCommTask} + $frame{AppComm_ServiceTelemetry}, 'AppComm_TrySendIrGet']
    if $has_ir_reply;
push @chains, ['8  USD,GET reply (telemetry)', $frame{StartCommTask} + $frame{AppComm_ServiceTelemetry}, 'AppComm_TrySendUsdGet']
    if exists $frame{AppComm_TrySendUsdGet};
push @chains, ['3d MOV,FIRL/FIRR request (early path)', $fir_prefix, 'on_move'];
push @chains, ['7  IREDGE, decision line (telemetry)', $frame{StartCommTask} + $frame{AppComm_ServiceTelemetry}, 'AppComm_TrySendIrEdge']
    if $has_edge_line;

printf "\nCommTask stack gate: stack=%d  reserve(exception+context)=%d  required slack=%d  => depth limit %d B\n",
       $STACK, $RESERVE, $MIN_MARGIN, $limit;
printf "%-38s %6s %8s %8s %8s  %s\n", 'chain', 'depth', 'raw mrg', '-reserve', '-slack', 'verdict';
sub margins { my ($t) = @_; return ($STACK - $t, $STACK - $t - $RESERVE, $STACK - $t - $RESERVE - $MIN_MARGIN) }
for my $c (@chains) {
    my ($name, $prefix, $root) = @$c;
    my ($d, $p) = depth($root);
    my $total = $prefix + $d;
    my ($m0, $m1, $m2) = margins($total);
    my $ok = ($total <= $limit);
    $fail++ unless $ok;
    printf "%-38s %6d %8d %8d %8d  %s\n", $name, $total, $m0, $m1, $m2, $ok ? 'ok' : 'FAIL (over budget)';
    print "      " . join(' -> ', @$p) . "\n" if $verbose;
}

# ------------------------------------------------------------------ MotionTask (informational)
# MotionTask's 1024 B stack (main.c) is not a gate here (its pre-existing worst chain, StartMotionTask ->
# AppMotion_Update, already leaves less than the 100 B slack after the reserve). The Phase 5 service
# (AppMotion_ServiceIrEdge, called by StartMotionTask next to AppMotion_Update, not inside it) is
# reported so that it can be seen to be shallower than that chain.
{
    my $motion_stack = 1024;
    my @mroots = grep { exists $frame{$_} } qw(StartMotionTask AppMotion_Update AppMotion_HandleCommand
                                              AppMotion_ServiceIrEdge AppIr_GetSnapshot);
    if (@mroots) {
        printf "\nMotionTask static depths (stack %d, reserve %d; informational):\n", $motion_stack, $RESERVE;
        for my $r (@mroots) {
            my ($d, $p) = depth($r);
            my $t = ($r eq 'StartMotionTask') ? $d : $frame{StartMotionTask} + $d;
            printf "  %-26s %5d B   raw margin %4d   after reserve %4d\n", $r, $t, $motion_stack - $t,
                   $motion_stack - $t - $RESERVE;
            print "      " . join(' -> ', @$p) . "\n" if $verbose;
        }
        if (exists $frame{AppMotion_ServiceIrEdge}) {
            my ($ds) = depth('AppMotion_ServiceIrEdge');
            my ($du) = depth('AppMotion_Update');
            check("FIR service chain is shallower than the Update chain (service $ds B <= update $du B)", $ds <= $du);
        }
    }
}

# ------------------------------------------------------------------ census
my @reach = sort keys %seen;
my @over_funcs;
for my $f (@reach) {
    my ($d) = depth($f);
    my $t = $entry{$f} + $d;
    push @over_funcs, [$t, $f] if $t > $limit;
}
# the global worst chain is the one rooted at StartCommTask itself
my ($worst_total, $root_p) = depth('StartCommTask');

# parser callbacks reached through the dispatcher (every `on_*` the parser can invoke)
my %skip = map { $_ => 1 } qw(on_stop on_us_park_get on_us_park_set on_ir_get on_usd_get);
my @cb;
for my $f (sort keys %frame) {
    next unless $f =~ /^on_/ && !$skip{$f};
    next unless has_edge('vision_rx_dispatch', $f);
    my ($d) = depth($f);
    push @cb, [$prefix_dispatch + $d, $f];
}
@cb = sort { $b->[0] <=> $a->[0] } @cb;
my @cb_over = grep { $_->[0] > $limit } @cb;

print "\ncensus of every chain reachable from StartCommTask:\n";
printf "  reachable functions                         : %d\n", scalar(@reach);
printf "  parser callbacks via vision_rx_dispatch()   : %d\n", scalar(@cb);
{
    my @show = ($census || $verbose) ? @cb : @cb[0 .. ($#cb < 4 ? $#cb : 4)];
    for my $r (@show) {
        my ($m0, $m1, $m2) = margins($r->[0]);
        printf "    %-32s %5d B  raw %5d  -reserve %5d  -slack %5d  %s\n", $r->[1], $r->[0], $m0, $m1, $m2, $r->[0] > $limit ? 'OVER' : 'ok';
    }
    print "    (top 5 shown; --census lists all)\n" unless $census || $verbose;
}
my ($w0, $w1, $w2) = margins($worst_total);
printf "  WORST CHAIN                                 : %d B   (%s)\n", $worst_total, join(' -> ', @$root_p);
printf "  raw margin (%d - %d)                      : %d B\n", $STACK, $worst_total, $w0;
printf "  margin after %d B exception/context reserve : %d B\n", $RESERVE, $w1;
printf "  margin after additional %d B slack          : %d B\n", $MIN_MARGIN, $w2;
printf "  over-budget chains (reachable functions)    : %d\n", scalar(@over_funcs);
printf "  over-budget parser callbacks                : %d of %d\n", scalar(@cb_over), scalar(@cb);
for my $o (sort { $b->[0] <=> $a->[0] } @over_funcs) { printf "    OVER %5d B  %s\n", $o->[0], $o->[1] }
check("every reachable chain is within the $limit B budget (worst $worst_total B)", $worst_total <= $limit && !@over_funcs);
check("no parser callback is over budget", !@cb_over);
check('call graph has no recursion in the CommTask-reachable set' . (@recursive ? ' (' . join('; ', @recursive) . ')' : ''), !@recursive);

# every reachable function with an unfollowed indirect call, so the model's blind spots are visible
my @ind = sort grep { $seen{$_} } keys %indirect;
print "  indirect-call functions in the reachable set : " . (@ind ? join(', ', @ind) : 'none') . "\n";
# every reachable callee must be resolved to a function body (no unknown library frames)
my %unknown;
for my $f (@reach) { for my $c (keys %{ $calls{$f} || {} }) { $unknown{$c} = 1 if !exists $frame{$c} && !$excl{$c} } }
check('no unresolved callees (every reachable bl target has a frame)' . (%unknown ? ': ' . join(', ', sort keys %unknown) : ''), !%unknown);

# ------------------------------------------------------------------ structure
print "\nstructural checks (what keeps the STOP/USPARK chains short):\n";
check('vision_rx_feed() calls on_stop() directly (exact STOP fast path)',        has_edge('vision_rx_feed', 'on_stop'));
check('vision_rx_dispatch() does NOT reference on_stop (no duplicate parsing / large frame)', !has_edge('vision_rx_dispatch', 'on_stop'));
check('vision_rx_feed() handles USPARK before the dispatcher',                   has_edge('vision_rx_feed', 'vision_rx_dispatch_us_park'));
check('vision_rx_dispatch() does NOT reference the USPARK handlers',             !has_edge('vision_rx_dispatch', 'on_us_park_get') && !has_edge('vision_rx_dispatch', 'on_us_park_set'));
check('vision_rx_feed() handles IR,GET before the dispatcher',                   has_edge('vision_rx_feed', 'vision_rx_dispatch_ir'));
check('vision_rx_dispatch() does NOT reference the IR,GET handler',              !has_edge('vision_rx_dispatch', 'on_ir_get'));
check("on_ir_get() only records a request (frame $frame{on_ir_get} B <= 16, no formatting)",
      $frame{on_ir_get} <= 16 && !has_edge('on_ir_get', 'sniprintf') && !has_edge('on_ir_get', 'snprintf'));
check('IR,GET reply is formatted by the telemetry service, never by the parser path',
      !$has_ir_reply || (has_edge('AppComm_ServiceTelemetry', 'AppComm_TrySendIrGet') && !has_edge('on_ir_get', 'AppComm_TrySendIrGet')));
check('vision_rx_dispatch() does NOT reference the USD,GET handler',             !has_edge('vision_rx_dispatch', 'on_usd_get'));
check("on_usd_get() only records a request (frame $frame{on_usd_get} B <= 16, no formatting)",
      $frame{on_usd_get} <= 16 && !has_edge('on_usd_get', 'sniprintf') && !has_edge('on_usd_get', 'snprintf'));
check('USD,GET reply is formatted by the telemetry service, never by the parser path',
      has_edge('AppComm_ServiceTelemetry', 'AppComm_TrySendUsdGet') && !has_edge('on_usd_get', 'AppComm_TrySendUsdGet'));
check('vision_rx_feed() handles MOV,FIRL/FIRR before the dispatcher',            has_edge('vision_rx_feed', 'vision_rx_dispatch_fir'));
check('vision_rx_dispatch() does NOT reference the FIR helper',                  !has_edge('vision_rx_dispatch', 'vision_rx_dispatch_fir'));
check('IREDGE, decision line is formatted by the telemetry service, never by the parser path',
      !$has_edge_line || (has_edge('AppComm_ServiceTelemetry', 'AppComm_TrySendIrEdge') && !has_edge('on_move', 'AppComm_TrySendIrEdge')));
check("on_stop() has no wrapper frame (frame $frame{on_stop} B <= 32, no call to AppComm_RequestPriorityStop)",
      $frame{on_stop} <= 32 && !has_edge('on_stop', 'AppComm_RequestPriorityStop'));
check("STOP helper keeps big structures off the stack (AppComm_RequestPriorityStop frame $frame{AppComm_RequestPriorityStop} B <= 64)",
      $frame{AppComm_RequestPriorityStop} <= 64);
check("telemetry service keeps big structures off the stack (AppComm_ServiceTelemetry frame $frame{AppComm_ServiceTelemetry} B <= 96)",
      $frame{AppComm_ServiceTelemetry} <= 96);
check('telemetry service does not nest the parking pass (AppComm_ServiceParking is a separate StartCommTask call)',
      !has_edge('AppComm_ServiceTelemetry', 'AppComm_ServiceParking') && has_edge('StartCommTask', 'AppComm_ServiceParking'));
check("vision_rx_feed() frame stays small ($frame{vision_rx_feed} B <= 32)", $frame{vision_rx_feed} <= 32);

unlink $tmp if defined $tmp;
print $fail ? "\nRESULT: FAIL ($fail)\n" : "\nRESULT: PASS\n";
exit($fail ? 1 : 0);
