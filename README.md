# SC2079 Multidisciplinary Design Project — Group 7

NTU SC2079 MDP (AY2026/27 Sem 1). An STM32 robot car that is driven by a
Raspberry Pi, planned by a laptop service, and controlled from an Android
tablet over Bluetooth.

- **Task 1:** image recognition. Visit up to 8 obstacles, read the symbol on
  each one, and show the results live on the tablet.
- **Task 2:** fastest car. Drive round two obstacles on the side shown by each
  arrow, then park back in the carpark.

## Repository layout

```text
task1/                 Task 1 (image recognition)
  stm/                 STM32F407 firmware (FreeRTOS, CubeIDE): motion, PID, IMU, UART commands
  algo/                Laptop path planner, HTTP + JSON (server.py, POST /plan/v2) and web UI
  rpi/                 Raspberry Pi program: Android <-> planner <-> STM, camera on SNAP
  android/             Android tablet app (Kotlin, Jetpack Compose, Bluetooth SPP)
  image_rec/           Image-recognition model and training code (placeholder)
task2/                 Task 2 (fastest car)
  stm/                 STM32 firmware with the Task 2 additions (IR, ultrasonic, USPARK parking)
  algo/                Notes only: the Task 2 planner is Python inside task2/rpi
  rpi/                 Raspberry Pi program final_run.py, its planner, simulator and tests
  android/             Android app with the Task 2 screen
  image_rec/           Arrow model and training code (placeholder)
PROTOCOL.md            Wire formats, coordinates and robot geometry shared by all subsystems
```

## System overview

```text
Android ──Bluetooth SPP── RPi ──Wi-Fi / HTTP── Laptop planner (task1/algo)
                           │
                           ├── Pi camera + YOLO (NCNN) image recognition
                           └── UART 115200 8N1 ── STM32F407 ── motors, servo, encoders, IMU, IR, ultrasonic
```

The RPi runs the mission. It sends the STM one `MOV` command at a time and
waits for `ACK` before sending the next. The STM does no planning; it carries
out each command and reports `ACK` or `ERR`. All the message formats are in
[`PROTOCOL.md`](PROTOCOL.md).

## Building and running

| Part | How |
|---|---|
| STM | Open the `.ioc` in STM32CubeIDE and generate code. This recreates `Drivers/` and `Middlewares/`, which are not included here. Then build and flash. Host tests: `gcc` in `stm/tests/` |
| Planner | `cd task1/algo && python server.py`, then run the tests with `python -m unittest discover -s tests -v` |
| RPi Task 1 | See [`task1/rpi/README.md`](task1/rpi/README.md) |
| RPi Task 2 | See [`task2/rpi/README.md`](task2/rpi/README.md) |
| Android | Open `android/` in Android Studio, then build and install |

## Notes

- Only the code the team wrote is included. Vendor HAL, CMSIS, FreeRTOS, build
  output, model weights and datasets are left out.
- `task2/rpi` is the Task 2 Pi code as of 15 Oct 2026. The final on-robot
  version may differ slightly.

## Team

Group 7

| Subsystem | Members |
|---|---|
| STM32 firmware | Sujith, Koulik Roy |
| Raspberry Pi and image recognition | Ash, Nam |
| Algorithm | Brandon |
| Android | Revathi, Xuehao |
