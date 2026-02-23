# Hardware architecture — real Mecanum AMR

This document describes the physical system this repository was designed for: a large mecanum-wheel AMR with lift, using **Jetson Nano** as the main computer and **STM32** as the motor-driver and I/O controller.

## Block diagram

```
                    ┌──────────────────────────────────────────────────┐
                    │              Jetson Nano                           │
                    │  Ubuntu 20.04, ROS 2 Humble                        │
                    │  Nav2, SLAM, QR, task manager                     │
                    │  mecanum_hw_bridge (UART <-> ROS)                 │
                    │  LiDAR, camera                                    │
                    └──────────────┬───────────────────────────────────┘
                                   │ UART (TX/RX, GND), 115200 baud
                                   ▼
                    ┌──────────────────────────────────────────────────┐
                    │              STM32 (e.g. F4 / F7)                  │
                    │  UART: parse motion + lift commands               │
                    │  PWM: 4x wheels, 1x lift                         │
                    │  Watchdog: stop if no command for N ms            │
                    └──────────────┬───────────────────────────────────┘
                                   │
             ┌─────────────────────┴─────────────────────┐
             ▼                                           ▼
      ┌─────────────┐                             ┌─────────────┐
      │ 4x Mecanum  │                             │ Lift        │
      │ motor driver│                             │ actuator    │
      └─────────────┘                             └─────────────┘
```

## Jetson Nano

- **Role:** Run full autonomy (mapping, localization, Nav2, QR, task state machine). Outputs `/cmd_vel` and lift service calls; the hardware bridge converts these to UART frames for the STM32.
- **Serial to STM32:** `/dev/ttyTHS1` (40-pin header) or USB-serial `/dev/ttyUSB0`. Use 3.3V or level shifter if STM32 is 3.3V.
- **Sensors:** LiDAR and camera on Jetson; ROS drivers publish `/scan` and `/camera/image_raw`.

## STM32

- **Role:** Low-level only. Receive motion and lift commands; drive motors and lift. Optional: encoder feedback or telemetry back to Jetson.
- **Safety:** Watchdog — if no valid command for 300–500 ms, stop all motors and lift.

## Connection summary

| From      | To    | Interface |
|-----------|-------|-----------|
| Jetson TX | STM32 RX | UART (commands) |
| Jetson RX | STM32 TX | UART (optional telemetry) |
| GND       | GND   | Common ground |
| STM32     | 4x motors | PWM / motor driver |
| STM32     | Lift  | GPIO/PWM |

Pinout and timer channels are board-specific; see [stm32/](stm32/) for protocol handling and placeholders.
