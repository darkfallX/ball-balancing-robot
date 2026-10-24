# 3-RRS Ball-Balancing Robot

A final-year mechatronics project that stabilises and moves a steel ball on a three-legged 3-RRS parallel platform. The system combines a Teensy 4.1, three NEMA 17 stepper motors, TMC2208 drivers, a four-wire resistive touch panel and a browser-based control station.

## What it does

- Balances a 25.4 mm steel ball at a selected set-point.
- Switches at runtime between PID, LQR, Mamdani fuzzy-logic and simplified model-predictive controllers.
- Generates circle, square, triangle, figure-eight, heart and line reference paths.
- Accepts tap-to-position commands from the browser control station.
- Streams live ball position, target position, controller selection and performance telemetry over serial.

## Hardware

| Item | Implementation |
| --- | --- |
| Platform | Three-revolute-revolute-spherical (3-RRS) parallel mechanism |
| Controller | Teensy 4.1 |
| Actuation | Three NEMA 17 bipolar stepper motors |
| Drivers | Three TMC2208 stepper-driver modules at 1/16 microstepping |
| Position sensing | 8.4-inch four-wire resistive touch panel |
| Ball | 25.4 mm steel bearing ball |
| Power | 24 V motor supply with a separate regulated 5 V logic rail |

## Repository layout

```text
.
├── firmware/
│   └── Ball_Balancer_Master/
│       ├── Ball_Balancer_Master.ino
│       ├── InverseKinematics.cpp
│       └── InverseKinematics.h
├── web/
│   └── BallBalancer_ControlStation.html
└── media/
    └── videos/                 # Add demonstration video here
```

## Firmware setup

1. Install the Arduino IDE with Teensy support.
2. Install the `AccelStepper` and `TouchScreen` libraries. `MultiStepper` is supplied with `AccelStepper`.
3. Open `firmware/Ball_Balancer_Master/Ball_Balancer_Master.ino`.
4. Select **Teensy 4.1** and the correct serial port.
5. Confirm the mechanism is at its manual level reference before enabling the motors.

The current implementation uses the following Teensy pins:

| Motor | STEP | DIR | ENABLE |
| --- | ---: | ---: | ---: |
| A | 22 | 23 | 21 |
| B | 19 | 20 | 18 |
| C | 40 | 41 | 39 |

The touch panel uses `A2`, `A3`, `A0` and `A1`.

## Control station

Serve the `web` directory with any local HTTP server, then open it in a Chromium-based browser such as Chrome or Edge. The interface uses the Web Serial API to communicate with the Teensy.

```bash
python -m http.server 8000 --directory web
```

Open `http://localhost:8000` and select the correct serial port.

## Serial protocol

Commands are newline terminated.

| Command | Function |
| --- | --- |
| `C,PID` / `C,LQR` / `C,FUZZY` / `C,MPC` | Select a controller |
| `G,<kp>,<ki>,<kd>` | Set PID gains |
| `K,<k1>,<k2>` | Set LQR gains |
| `F,<eMax>,<dMax>,<uMax>` | Set fuzzy-controller scales |
| `M,<q1>,<q2>,<r>,<N>` | Set MPC weights and horizon |
| `I,<kiTrim>` | Set non-PID integral trim |
| `P,<shape>[,<size>][,<period>][,<ramp>]` | Select or configure a path |
| `R,<absoluteX>,<absoluteY>` | Move to an absolute plate position |
| `W,<gain>` | Set feedforward gain |
| `B,<biasX>,<biasY>` | Set feedforward tilt bias |
| `T,<nx>,<ny>[,<seconds>]` | Apply a temporary manual tilt |
| `S` | Request system status |

## Safety

This is a physical motion-control system. Keep hands clear of the mechanism while the motors are enabled, verify the 24 V and 5 V rails before use, and begin testing with conservative output limits and a clear working area.

## Mechanical-model acknowledgement

The mechanical 3D model used for fabrication was based on the open-source ball-balancer model published by [Aaed Musa](https://www.instructables.com/Ball-Balancer/). This acknowledgement applies to the mechanical model only. The inverse kinematics, firmware, controller implementations, trajectory logic, pin allocation, browser control station and experimental work in this repository are specific to this project.

## Demonstration video

A project demonstration video can be added to `media/videos/`. The folder is intentionally retained in Git through `.gitkeep`.
