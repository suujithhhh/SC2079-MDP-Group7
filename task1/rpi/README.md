# RPi Task 1 program

Connects Android, the laptop planner ([`../algo/`](../algo/), `POST /plan/v2`) and
the STM for Task 1, following [`PROTOCOL.md`](../../PROTOCOL.md). Hardware-free unit
tests are in `tests/`. Model weights (`mdp_circle_v2_ncnn_model/`, `best.pt`) are not
included in this repo; copy them next to `task1.py` on the Pi.

| File | Role |
|---|---|
| `task1.py` | Entry point and mission sequencing; `--map` bench mode without the tablet |
| `android_link.py` | Bluetooth SPP/RFCOMM server (stdlib socket, no PyBluez), line framing, reconnect |
| `planner_client.py` | Android `PLAN_REQUEST` → `POST /plan/v2`; start corner → rear axle; pose → `ROBOT` |
| `uart.py` | Serial transport only: opens the port (pyserial, 115200 8N1), writes one LF-terminated ASCII line, buffers/frames received bytes into lines, disconnect and shutdown handling. No STM commands, no reader thread, never resends |
| `stm_link.py` | The STM protocol on top of `uart.py`: one `MOV` in flight, exact `ACK` / `ACK,STOPPED` (status "stopped": not success, pose unknown) / `ERR,...` terminal, telemetry filtered (`USPARK,STOP`/`USPARK,FAULT` cause lines exposed on the reply), `PROFILE`/`IMU,GET`, `USPARK,1`/`0`/`GET` (`uspark_enable`/`uspark_disable`/`uspark_get`), never resends |
| `vision.py` | Pi camera + NCNN model on `SNAP`, stitched result image |
| `mdp_circle_v2_ncnn_model/` | Current NCNN model (YOLO26s-seg, classes 11-40 + marker, 640 px); the `--model` default. Its `best.pt` is the source weights. The previous model is kept as `best_ncnn_model_v3_old/` (pass `--model best_ncnn_model_v3_old` to use it) |
| `tests/` | Hardware-free unit tests |

## Start / READY handshake

The robot never moves on Submit. After the plan comes back the Pi sets the STM
up (`PROFILE,PRECISION` + `PROFILE,GET`, and an `IMU,GET` reading while the
heading fix is on), then sends `READY` and `MSG,[Ready: N moves, press Start]`
and waits. `START` runs the plan, once; resubmit the map to run again.

- A `START` that arrives before `READY` (Android shows Start right after
  Submit, and planning takes up to ~70 s) is ignored with
  `MSG,[START ignored: not ready yet, wait for READY]`; nothing moves.
- If setup fails, no `READY` is sent: `MSG,[Not ready: <reason>]`.
- `READY` is sent again when the tablet reconnects while a plan is armed or a
  mission is paused.
- `--start-on-plan` moves as soon as the plan is ready (the old behaviour).
  `--map` still waits for Enter; `--listen` still moves as soon as a plan is
  pushed.

See [`ANDROID_START_READY.md`](ANDROID_START_READY.md) for the Android side.

## Planner

`/plan/v2` plans with the STM's four measured turn radii and the measured
robot footprint, and returns each move as its exact STM string
(`stm_command`), which the Pi sends unchanged after the plan's `stm_setup`.
It accepts 1-8 obstacles and returns within a bounded 70 s window (B3, the
default mode, uses about 63 s of it).

## Messages

| Direction | Message |
|---|---|
| Android → Pi | `PLAN_REQUEST` JSON line (Submit), then `START` once the Pi has sent `READY` (or `RESUME` after a disconnect) |
| Pi → Android | `READY` (plan received, STM set up, waiting for `START`), `MSG,[...]`, `ROBOT,<x>,<y>,<N/E/S/W>` after each `ACK`, `TARGET,B<obstacle>,<image id>` after each `SNAP` |
| Pi → planner | `POST /plan/v2`, cm units, `mission_id` per plan |
| Pi → STM | the plan's `stm_setup` (`PROFILE,...`, `PROFILE,GET`), then each `stm_command` |

Positions: `start` is the robot's **rear-axle midpoint** (the planner's and
`PROTOCOL.md` §1's reference point) in the request's `coordinate_units`,
e.g. `20, 10` in cm or `2, 1` in grid; it is passed to the planner unchanged.
Obstacles are given by their bottom-left corner. `ROBOT` updates report the
bottom-left cell of Android's 3×3 box, assumed centred on the robot body.

## Run

All planning runs on the laptop (`algo/server.py`); the Pi only sends
`POST /plan/v2` and executes the result, so `algo/` is not needed on the Pi.

On the laptop, from `algo/`:

```sh
python3 server.py            # listens on 0.0.0.0:5000
```

On the Pi, pass the laptop's IP (`hostname -I` on the laptop); port 5000 and
`/plan/v2` are filled in, or give `IP:port` / a full URL.

Bench test without the tablet (Bluetooth unused): plans, prints the moves,
waits for Enter, then runs.

```sh
~/mdp/.venv/bin/python task1.py --planner <laptop-ip> --map map.json
```

With the tablet (the checkpoint bridge owns RFCOMM channel 1, so stop it first):

```sh
sudo systemctl stop rfcomm
~/mdp/.venv/bin/python task1.py --planner <laptop-ip>
```

Every request is protocol v2 in mode B3: the Pi ignores the `protocol_version` and `mode`
Android sends and builds its own request (`--mode B2` is for testing only). `--planner` only
accepts the `/plan/v2` path (a bare IP gets it added; `/plan`, the old v1 endpoint, is refused).

Alternative viewing poses are on by default (`--no-alt-viewing` turns them
off); without them multi-obstacle maps often have no route. The server answers
within its 70 s request ceiling; the Pi waits up to `--planner-timeout` (75 s).
Results and the stitched image go to `~/mdp/task1_runs/<mission_id>/`. Every line
sent to and received from the STM, including all `MM,` movement telemetry, is
appended to `~/mdp/task1_runs/stm_telemetry.log` (one `## mission <id>` header
per run); the console shows one heading/wheel summary per move.

`--listen PORT` is the other direction: the laptop pushes finished plans to
the Pi (`POST /run`) and `--planner` is not needed.

## STM serial link

The STM is wired to the Pi, and the Python process on the Pi (started over SSH)
opens the device named by `--stm` (default `/dev/ttyACM0`): 115200 baud, 8 data
bits, no parity, 1 stop bit, no flow control, ASCII, one `\n`-terminated line
per command. `/dev/ttyACM0` can change when the board is replugged; for
deployment pass the stable path instead (list it with `ls -l /dev/serial/by-id/`),
and make sure the Pi user may open it (usually the `dialout` group).

```sh
cd ~/mdp/rpi_task1
~/mdp/.venv/bin/python task1.py --planner <laptop-ip> --stm /dev/serial/by-id/<name-shown-by-ls>
```

(stop `rfcomm` first when using the tablet, see Run.) A transport failure
(device missing, unplugged, write error, garbled stream) ends the current
command and the mission with the robot position marked unknown; nothing is
resent and nothing reconnects by itself.

## Tests

```sh
cd rpi && python -m unittest discover -s tests -v
```

## Known limits

- The laptop must be reachable from the Pi; if it is not, the plan fails and nothing moves.
- Any STM error or timeout ends the mission; nothing is resent.
- The `TARGET` obstacle field uses `B<n>` (Android spec); Android also accepts a bare number.
