# STM32 motor-driver firmware interface

This folder describes and implements the **STM32 side** of the Jetson–STM32 link: UART receive, protocol parsing, and placeholders for motor and lift control. The actual pinout, timer channels, and motor drivers are board-specific; here we provide the communication layer and integration points.

## Role of the STM32

- Receive ASCII frames from the Jetson over UART (see [../docs/PROTOCOL.md](../docs/PROTOCOL.md)).
- **Motion:** On `M,w_fl,w_fr,w_rl,w_rr`, set wheel motor targets (PWM or velocity closed loop).
- **Lift:** On `L,0` / `L,1`, drive the lift actuator down/up.
- **Watchdog:** If no valid frame is received for a configured timeout (e.g. 300–500 ms), stop all motors and lift.

## Contents

| File / folder | Description |
|---------------|-------------|
| [Core/](Core/) | Protocol parser and command handlers (platform-agnostic C). |
| [Core/protocol.c](Core/protocol.c) | UART line buffer and parsing (M and L commands). |
| [Core/protocol.h](Core/protocol.h) | API for the parser and callbacks. |
| [Core/motor_lift_stub.c](Core/motor_lift_stub.c) | Stub implementations: replace with your HAL/PWM/encoder code. |
| [main.c](main.c) | Example main loop with watchdog (integrate with your HAL). |
| [main_host.c](main_host.c) | Host test: parses sample frames without hardware. |
| [Makefile](Makefile) | Build protocol + stub for host: `make build_host` then `./main_host`. |

## Integration

1. **UART:** Configure your HAL (e.g. STM32Cube) for the chosen USART (e.g. USART2), 115200 8N1. In the RX complete or idle callback, feed received bytes into `protocol_feed_byte()`.
2. **Callbacks:** Implement `motor_set_speeds(w_fl, w_fr, w_rl, w_rr)` and `lift_set(int up)` (or use the stubs and replace later).
3. **Watchdog:** A timer or main-loop check: if `protocol_last_rx_time()` is older than the timeout, call `motor_set_speeds(0,0,0,0)` and `lift_set(0)`.

The provided code is intended to be dropped into an existing STM32 project (CubeIDE, Makefile, or other) and does not include startup files or HAL drivers.
