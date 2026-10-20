# START / READY handshake: Pi changes and what Android needs to do

Written 2026-10-06. Covers the Pi side (done) and the Android side (todo).
Based on Android commit `d80814a` ("UI update to include START") on `origin/android`.

## Summary

The robot no longer moves when the map is submitted. The Pi plans the route,
sets the STM up, sends **`READY`**, and then waits. The robot starts moving only
when Android sends **`START`**.

```
Android                          Pi                               laptop planner / STM
  | PLAN_REQUEST (Submit) ------>|                                        |
  |                              |-- POST /plan/v2 ---------------------->|  (up to ~70 s)
  |<----- MSG,[Planning route..] |<-- command list -----------------------|
  |                              |-- PROFILE,PRECISION / PROFILE,GET ---->|  set up the STM,
  |                              |-- IMU,GET -------------------------->  |  nothing moves
  |<----- READY                  |                                        |
  |<----- MSG,[Ready: N moves, press Start]                               |
  | START ----------------------->|                                       |
  |<----- MSG,[Mission started]  |-- MOV ... (robot moves) -------------->|
  |<----- ROBOT,x,y,D / TARGET,Bn,id (as before)                          |
  |<----- MSG,[Mission done in Ns]                                        |
```

## Wire messages (one line each, ending in `\n`)

| Direction | Line | Meaning |
|---|---|---|
| Android → Pi | `PLAN_REQUEST` JSON | Submit (unchanged) |
| Pi → Android | **`READY`** | **New.** Plan received, STM set up, the robot will move as soon as `START` arrives |
| Android → Pi | **`START`** | Move now. Only acted on after `READY`. Case-insensitive, whitespace trimmed. `RESUME` is accepted as a synonym |
| Pi → Android | `MSG,[Ready: N moves, press Start]` | Human-readable twin of `READY`, sent right after it |
| Pi → Android | `MSG,[START ignored: not ready yet, wait for READY]` | `START` arrived before `READY` (for example during planning). The robot did not move |
| Pi → Android | `MSG,[START ignored: no plan ready, submit the obstacle map first]` | `START` with no armed plan (nothing submitted, planning failed, or the plan already ran) |
| Pi → Android | `MSG,[Not ready: <reason>]` | STM setup failed (profile or IMU). No `READY` is sent. Check the Pi / STM, then Submit again |
| Pi → Android | `MSG,[Pi connected: send the obstacle map]` | Tablet (re)connected and no plan is armed (was `Pi ready: ...`; reworded so it cannot be confused with `READY`) |

Unchanged: `ROBOT,<x>,<y>,<N/E/S/W>`, `TARGET,B<n>,<id>`, `MSG,[...]`.

## Pi behaviour to rely on

- **`READY` means `START` will move the robot.** The STM profile is already set
  and verified (`PROFILE,PRECISION`), and the IMU has answered.
- **One run per `READY`.** After a run starts, a further `START` is ignored. To
  run again, Submit again (new plan, new `READY`).
- **Submit again replaces the plan.** A new `PLAN_REQUEST` throws the armed plan
  away. `READY` only comes back after the new plan is set up. If a second
  `PLAN_REQUEST` arrives while the Pi is still planning the first, the newer one wins.
- **`READY` is repeated** when the tablet reconnects while a plan is armed.
  Also when the tablet reconnects while a mission is **paused** (the tablet
  dropped mid-run; the Pi finishes the current move, then waits for `START`
  to resume). So after any reconnect `READY` means "tap Start to go (again)".
- **Planning takes up to ~70 s** (usually ~64 s). `READY` arrives only after that
  plus a second or two of STM setup.
- **No `READY` means no `START` will work.** The Pi drops anything sent before it.

## What Android should do (spec below; already on the Android branches, not yet run on the tablet)

**Status 2026-10-07:** the branches `origin/android` and `origin/android-type-obstacles-text-entry`
already parse `READY` (`ProtocolParser.kt`) and keep an `isReady` flag that is passed to the Start
button (`ArenaViewModel.kt`, `ObstacleCreationBar.kt`). Checked from source only. Confirm the tablet
build includes this. If it does, the "Until Android is updated" workaround below is not needed.
The list below is the original spec.

Goal: the Start button turns **green only after `READY`**, and is grey/disabled
otherwise.

1. **Parse `READY`.** `ProtocolParser.parse` currently returns `null` for any verb
   other than `ROBOT`, `TARGET`, `MSG`, so a bare `READY` is dropped silently.
   Add `"READY" -> InboundMessage.Ready` (a bare verb with no fields; the existing
   verb splitting already copes with a line that has no comma). Add a parser test
   (`"READY"` and `"ready"` parse to `Ready`).
2. **Add a UI flag**, for example `isReady: Boolean = false` in `ArenaUiState`.
   - Set `isReady = true` and `isStartSent = false` when `Ready` arrives.
     (Resetting `isStartSent` is what makes Start usable again after a
     reconnect to a paused mission.)
   - Set `isReady = false` when: Submit is tapped, Edit Grid / Clear grid is
     tapped, the Bluetooth link drops (the Pi sends `READY` again on reconnect).
3. **Start button** (`StartButton` in `ObstacleCreationBar.kt`): keep showing it
   when `LOCKED && isPlanSent`, but make it **green and tappable only when
   `isReady && !isStartSent`**; otherwise grey and not clickable. Today it is
   green and tappable right after Submit.
4. **`onStartClick`**: only set `isStartSent = true` if the message was
   actually sent (today it greys the button even when Bluetooth is
   disconnected and nothing went out). Once `START` is sent, set
   `isReady = false`.
5. **Optional:** surface `MSG,[Not ready: ...]` prominently (the status log is
   enough) so the operator knows to check the Pi instead of waiting for `READY`.

Nothing else on the Android side needs to change: `START` is already sent as the
bare line `START` (`ProtocolSerializer.start()`), which is exactly what the Pi
expects.

### Until Android is updated

The tablet ignores `READY`, so the Start button is green and tappable straight
after Submit. That is safe (the Pi ignores an early `START`; the robot cannot
move) but the early tap greys the button, and the only way to get it back is
Edit Grid, then Submit, which replans (another ~70 s). So for now:
**wait for `Ready: N moves, press Start` in the status log before tapping Start.**

## What changed on the Pi (`rpi_task1/`)

- `task1.py`
  - New `Mission.prepare()` / `setup_stm()`: set + verify the STM profile (and
    check `IMU,GET` while the heading fix is on) right after the plan arrives,
    then arm the plan and send `READY`. On failure: `MSG,[Not ready: ...]`, nothing armed.
  - `START` runs the armed plan once. The profile is not set again at `START`.
  - `START` / `RESUME` that arrive while planning or setting up are answered with
    "START ignored: not ready yet" and dropped, so they cannot trigger a run later.
  - A newer `PLAN_REQUEST` received during planning replaces the current one.
  - `READY` is re-sent on reconnect while armed, and on reconnect during a paused mission.
  - New flag `--start-on-plan` (move as soon as the plan is ready, the old
    behaviour). The old default is now the opposite, so `--wait-for-start` is
    accepted and does nothing. `--map` (waits for Enter) and `--listen` (pushed
    plans run at once) behave as before; `--map` now sets the STM up before the Enter prompt.
  - The run loop itself (MOV / SNAP / heading fix / vision) is untouched.
- `tests/test_rpi.py`: 14 new tests in `StartReadyTests` (READY only after
  setup; no motion before START; START runs once; START during planning
  ignored; newer request wins; no READY on profile/IMU/planner failure;
  READY re-sent on reconnect and in a paused mission; `--start-on-plan`).
- `README.md`: "Start / READY handshake" section and the messages table.

The planner, the STM firmware and the motion profile (PRECISION) are not
touched. The profile commands are the same, they just go out before `READY`
instead of after `START`.

## Test status

- **Verified:** hardware-free unit tests, 63 pass (49 existing + 14 new),
  `python -m unittest discover -s tests` from `rpi_task1/`.
- **Not verified (needs the robot and the tablet):**
  1. `READY` and the `Ready: ...` message appear on the tablet after Submit.
  2. Tapping Start before `READY` does not move the robot.
  3. Tapping Start after `READY` moves the robot, with no pause for profile setup.
  4. `READY` comes back after a Bluetooth reconnect (armed plan, and paused mission).
  5. The robot sitting idle for a while between `READY` and `START` causes no
     STM or IMU problems.

## Open point to agree

`PROTOCOL.md` in the repo does not define the Android ↔ Pi `READY` / `START`
exchange yet. If Android adopts `READY`, it should be added there (wire
formats are frozen in that file) in the same change.
