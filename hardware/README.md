# Hardware integration — real robot stack

This folder contains the **hardware interface** used for the physical Mecanum AMR: **Jetson Nano** (ROS 2, high-level autonomy) and **STM32** (motor drivers, lift, low-level I/O). The simulation in this repo mirrors the same logic; here we document and provide the code that runs on the real platform.

## Stack overview

| Component   | Role |
|------------|------|
| **Jetson Nano** | Runs ROS 2 Humble: Nav2, SLAM, QR/task manager, kinematics. Publishes `/cmd_vel` and lift commands. Talks to STM32 over UART. |
| **STM32**        | Receives motion and lift commands via UART. Drives 4× mecanum motors (PWM/encoder feedback) and lift actuator. Optionally reports telemetry (currents, encoder counts). |

```
  ┌─────────────────────────────────────────────────────────────┐
  │  Jetson Nano (Ubuntu 20.04 / ROS 2 Humble)                  │
  │  Nav2, SLAM, QR, task manager  →  /cmd_vel, /lift/up|down  │
  │  mecanum_hw_bridge (this repo) →  serial port               │
  └───────────────────────────────────┬─────────────────────────┘
                                      │ UART (e.g. /dev/ttyTHS1 or USB‑serial)
                                      ▼
  ┌─────────────────────────────────────────────────────────────┐
  │  STM32 (motor driver MCU)                                    │
  │  Parses protocol → PWM for 4 wheels + lift                   │
  └─────────────────────────────────────────────────────────────┘
```

## Contents

| Path | Description |
|------|-------------|
| [HARDWARE_ARCHITECTURE.md](HARDWARE_ARCHITECTURE.md) | Block diagram, connections, Jetson and STM32 roles. |
| [docs/PROTOCOL.md](docs/PROTOCOL.md) | UART frame format: motion commands, lift, optional telemetry. |
| [jetson/](jetson/) | Notes and link to the ROS 2 node (package `mecanum_hw_bridge` in repo `src/`). |
| [stm32/](stm32/) | STM32-side C code: protocol parser and stubs for motor/lift control. |

## Quick start (on Jetson, with real robot)

1. Connect STM32 via UART (on-board `ttyTHS1` or USB–serial adapter, e.g. `/dev/ttyUSB0`).
2. Build and source the workspace (including `mecanum_control` for kinematics).
3. Run the hardware bridge with the correct port and baud rate:

```bash
ros2 launch mecanum_hw_bridge real_robot_bringup.launch.py serial_port:=/dev/ttyTHS1
```

4. Run your autonomy stack as usual; `/cmd_vel` and lift service calls will be forwarded to the STM32.

**Full step-by-step (STM32 + Jetson + first test):** [docs/REAL_ROBOT_QUICKSTART.md](../docs/REAL_ROBOT_QUICKSTART.md).  
Jetson-only details: [docs/DEPLOYMENT_JETSON.md](../docs/DEPLOYMENT_JETSON.md).
