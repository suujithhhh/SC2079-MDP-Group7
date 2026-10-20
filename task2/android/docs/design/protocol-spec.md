# Bluetooth Protocol Reference

Derived from the ARCM briefing slides (A/P Goh Wooi Boon, 7 Jan 2025) and the
CE/CZ3004 deliverable checklist. Put this in `docs/design/protocol.md` and
reference it from the project notes.

Everything marked **[given]** comes from the course material and must be
implemented exactly. Everything marked **[ours]** is yours to define — agree it
with the robot and algorithm teams before implementing.

---

## Transport

- Bluetooth Classic, SPP / RFCOMM
- UUID `00001101-0000-1000-8000-00805F9B34FB`
- Messages are text strings, one per line
- **Framing:** terminate outbound messages with `\n`. Buffer inbound bytes until
  a terminator is seen — RFCOMM is a byte stream, so a single `read()` may return
  half a message or two whole ones. Accept both `\n` and `\r\n` on input.

### Role: you need both client and server

The AMD tool connects **to** your tablet — its title bar reads "CONNECTED TO
Nexus 7" and its Bluetooth menu offers *Scan For Devices* and *Reconnect to
last device*. So your app must accept incoming connections.

But C.2 requires your GUI to initiate scanning, selection and connection. So
your app must also connect outward.

Implement both:

- `BluetoothServerSocket` via `listenUsingRfcommWithServiceRecord(name, UUID)`,
  in an accept loop that restarts after each disconnect
- `BluetoothSocket` via `device.createRfcommSocketToServiceRecord(UUID)` for
  the user-initiated path

Whichever wins, hand the resulting socket to a single connected-session handler.
This is the structure of Android's classic BluetoothChat sample
(AcceptThread / ConnectThread / ConnectedThread) — worth reading before you
start, though you'll write it with coroutines rather than raw threads.

**C.8 depends on the accept loop.** When AMD disconnects and reconnects, nothing
on your side initiates — you must already be listening. Test exactly this:
connect, then Disconnect from AMD's menu, then Connect again, without touching
the tablet.

---

## Inbound — robot/AMD to app

### `ROBOT,<x>,<y>,<direction>` **[given]**

Update robot position and facing. Direction is one of `N`, `S`, `E`, `W`.
Coordinates are integers in arena space.

```
ROBOT,7,2,N
```

### `TARGET,<obstacleId>,<targetId>` **[given]**

Replace the obstacle's number with the target ID, in large white text.

```
TARGET,B2,11
```

### `TARGET,<obstacleId>,<targetId>,<face>` **[given]**

Same, plus the face on which the image was found — drawn as a thick line of a
distinguishing colour on that edge.

```
TARGET,B2,11,N
```

Both arities appear in the slides. The checklist text mentions only the
three-field form. **Support both.**

### `MSG,[<text>]` **[given]**

Text to display in the status message box. Note the square brackets around the
payload.

```
MSG,[Moving]
MSG,[Looking for target 2]
```

This is also your answer to C.4's requirement that the message box show
*selective* information rather than the whole stream: display `MSG` payloads
and your own app-generated link events, never raw incoming lines.

---

## Outbound — app to robot

### `ADD,<obstacleId>,(<x>,<y>)` **[given]**

Obstacle placed, or an existing obstacle moved to new coordinates. Sent on
finger-lift, not continuously during the drag.

```
ADD,B1,(10,6)
```

### `SUB,<obstacleId>` **[given]**

Obstacle removed — sent when it's dragged outside the arena and released.

```
SUB,B1
```

### `FACE,<obstacleId>,<direction>` **[given]**

Target face annotated on an obstacle.

```
FACE,B2,N
```

### `PLAN_REQUEST` (JSON) **[ours]**

Sent once when Submit is pressed (`EditMode.EDITING -> LOCKED`), replacing `ADD`/
`SUB`/`FACE` for this flow per team decision — see
the team design notes.
Carries the full current arena state as one compact-JSON line (still `\n`-terminated
like every other outbound message):

```
{"type":"PLAN_REQUEST","protocol_version":1,"mode":"B2","coordinate_units":"grid","start":{"x":2,"y":2,"heading":"N"},"obstacles":[{"id":1,"x":6,"y":10,"face":"N"}]}
```

`start` is the robot's footprint bottom-left cell (see "Robot footprint" below) and
heading. `obstacles[].id` is a bare int, not `B`-prefixed. Submit is refused (stays
in `EDITING`) if any obstacle has no face set.

`ADD`, `SUB`, and `FACE` above remain documented as **[given]** — required by the
course material — but are no longer sent by this app for obstacle submission.
**Confirm with your supervisor whether the C-section grading checklist requires
literal `ADD`/`SUB`/`FACE` traffic independent of what the algorithm team's planner
consumes**; if so, both may need to be sent.

### Movement commands **[ours]**

Not specified anywhere in the course material. Agree the vocabulary with the
robot team — they have to parse it. A reasonable starting proposal:

```
MOVE,F     forward
MOVE,B     reverse
TURN,FL    front-left L-shaped turn
TURN,FR    front-right L-shaped turn
TURN,BL    back-left L-shaped turn
TURN,BR    back-right L-shaped turn
```

Turning is no longer in-place rotation: each `TURN,<code>` both displaces the
robot's bottom-left reference cell and rotates its heading 90°, per
the team design notes's geometry table.
`TURN,L`/`TURN,R` (in-place rotation) are no longer sent by this app.

Do not guess and implement in isolation. This is the one place where a
mismatch blocks integration.

---

## Obstacle identifiers

Obstacles are numbered from 1 as they're added, and referenced on the wire as
`B<n>` — `B1`, `B2`, `B3`. The operator can manually override the
auto-assigned id at creation time (see
the team design notes), so other
sub-teams should treat `B<n>` as an opaque key rather than assume a dense
`1..N` range.

Be liberal on input: accept `B2` and bare `2` for the same obstacle, since the
checklist wording says "<Obstacle Number>" while the slides show `B2`. Be strict
on output: always emit the `B`-prefixed form.

---

## Coordinates

Origin is **bottom-left**, with y increasing upward — the C.5 illustration
labels its vertical axis 0 at the bottom.

Compose's canvas has y increasing *downward*. Your `gridToPixel` must flip:

```kotlin
val px = originX + x * cellSize
val py = originY + (gridHeight - 1 - y) * cellSize
```

Get this into a pair of tested helper functions early. Nearly every map bug
traces back to a coordinate conversion.

The robot's footprint is 3x3 cells — square, so it doesn't change shape with
heading. `ROBOT,<x>,<y>,<direction>` and `start` in `PLAN_REQUEST` both report
the footprint's **bottom-left cell**, not its centre. The robot's physical
turning radius is 3 cells; see `turnedPose` in `ArenaGeometry.kt` and
the team design notes for how L-shaped
turns use it.

---

## Parser rules

- Split on the line terminator, then on `,`
- Trim whitespace around every field
- Match the verb case-insensitively
- Unknown verb, wrong arity, or unparseable field → log and drop the line.
  Never crash, never surface it to the user as a status message.
- A `TARGET` for an obstacle that doesn't exist → drop it, log it
- Malformed input is expected during integration; robustness here saves you
  during the demo

---

## Confirm with your team before building

These are unspecified in the material you have. Get answers before Phase 6.

1. **Arena dimensions.** The C.5 mockup shows roughly 13 × 14 cells, but that
   may be an illustration rather than the real arena. This determines `cellSize`
   and whether direct face-tapping on an obstacle is viable.
2. **Robot footprint — resolved.** The footprint is a 3x3-cell square, and
   `ROBOT,<x>,<y>,<direction>` reports the bottom-left cell, not the centre —
   see "Robot footprint" under Coordinates above.
3. **Movement command vocabulary** — see above.
4. **Line terminator** the robot actually sends.
5. **Whether the robot echoes** commands back, which would affect your status
   feed filtering.
6. **Whether the AMD tool or the robot initiates** the connection in your team's
   setup, so you know which path gets exercised in the demo.
7. **`PLAN_REQUEST` transport.** Does it travel over the same Bluetooth RFCOMM link
   as `ROBOT`/`TARGET`/`MSG`, or a separate channel to a different algorithm-team
   device?
8. **`mode` field.** What does `mode: "B2"` mean, and can it ever be a value other
   than `"B2"` from this app?
9. **3x3 footprint confirmation.** Confirm the 3x3-cell square, bottom-left-reference
   convention with the robot team — it changes what inbound `ROBOT,<x>,<y>,<direction>`
   means too.
10. **`PLAN_REQUEST` reply.** Does the algorithm team reply with a distinct message
    type, or does the plan come back as ordinary `ROBOT`/`MSG` traffic?
11. **`TURN,FL`/`FR`/`BL`/`BR` vocabulary and the L-shaped displacement model
    itself.** Both are `[ours]`/unconfirmed, per the design spec in
    the team design notes. Confirm the
    wire codes and the geometry (lateral shift = forward/backward shift =
    the turning radius, `ArenaConfig.ROBOT_TURN_RADIUS_CELLS` = 3 cells)
    with the robot team before relying on this for the demo.
12. **Swept-path collision approximation.** A turn's collision check treats
    the swept region as the union of the footprint at the turn's start pose
    and its end pose — not a continuous simulation of the path in between.
    An obstacle sitting in the gap between the two footprints (most of a
    turn's actual swept area) is invisible to this check. Confirm with the
    robot team whether this approximation is acceptable, or whether a real
    chassis can clip an obstacle this check would miss.
13. **Turn displacement moves the origin and the chassis centre by the same
    amount.** The robot has a constant 3x3 square footprint at every heading,
    so the chassis centre is always offset from the bottom-left origin cell
    by the same `(+1.5, +1.5)` — that offset does not change across a turn,
    so origin-cell displacement and centre displacement are identical (e.g. a
    front-right turn from NORTH moves both the origin and the centre by
    `(+3, +3)`). This resolves the origin-vs-centre displacement concern that
    applied under the old, non-square footprint. The only remaining question
    is which of the two points (`ROBOT,<x>,<y>,<direction>`'s bottom-left
    cell, or the geometric centre) the robot team's own position/turn
    reporting uses — already covered by item 9 above.
14. **Obstacle id uniqueness across the whole match, not just one grid
    session.** The operator can manually type any 2-digit id (1-99) for a
    new obstacle, not just the auto-assigned smallest-unused-gap value.
    Confirm with the robot/algo team whether obstacle ids need to be unique
    only within one `PLAN_REQUEST` snapshot (current behavior) or need any
    additional constraint (e.g. matching a specific physical numbering
    scheme printed on the arena's obstacles) before relying on manually
    assigned ids for the demo.

---

## Assessment note

Section C is signed off item by item, with a supervisor signature and a named
student contributor per row. Each of C.1–C.10 needs to be demonstrable on its
own.

Two consequences:

- Keep a `FakeTransport` in the build so you can demonstrate map items (C.5–C.7,
  C.9, C.10) without a working robot. Several items are provable with the AMD
  tool alone.
- Get items signed off as they're finished rather than saving them all for the
  end. C.1 through C.4 are signable as soon as Phase 8 works; the map items
  follow. Don't let sign-off block on integration with the other modules.
