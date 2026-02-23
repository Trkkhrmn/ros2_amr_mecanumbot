# UART protocol — Jetson ↔ STM32

Communication between Jetson Nano (ROS 2) and STM32 is over a single UART link. Default baud rate: **115200**, 8N1.

## Frame format (ASCII, line-based)

Each message is one line ending with `\n` (LF). This keeps parsing simple on both sides and allows debugging with a serial terminal.

### 1. Motion command (periodic, from Jetson → STM32)

Body velocity from ROS `/cmd_vel` is converted to wheel speeds on the Jetson; the STM32 receives **wheel angular speeds** so it does not need to run inverse kinematics.

```
M,w_fl,w_fr,w_rl,w_rr\n
```

- `M` = motion.
- `w_fl`, `w_fr`, `w_rl`, `w_rr` = floating-point wheel speeds in **rad/s** (front_left, front_right, rear_left, rear_right).
- Example: `M,1.2,-0.5,0.8,-0.3\n`

Sent at a fixed rate (e.g. 20–50 Hz). If the STM32 does not receive a motion frame within a timeout (e.g. 300–500 ms), it should stop all wheels (watchdog).

### 2. Lift command (on request, Jetson → STM32)

```
L,0\n   → lift down (release)
L,1\n   → lift up (grasp)
```

- `L` = lift.
- Second field: `0` = down, `1` = up.

The STM32 drives the lift actuator accordingly (e.g. relay or PWM) and may optionally reply with an acknowledgment (see Telemetry).

### 3. Telemetry (optional, STM32 → Jetson)

If the STM32 sends back data, each line can carry e.g. wheel speeds or status:

```
T,w_fl,w_fr,w_rl,w_rr,lift_pos\n
```

- `T` = telemetry.
- `w_*` = measured or estimated wheel speeds (rad/s).
- `lift_pos` = lift position (e.g. 0 = down, 1 = up) or raw ADC value.

The Jetson bridge can publish these to ROS (e.g. custom message or `Float64MultiArray`) for diagnostics or odometry.

### 4. Heartbeat / keep-alive (optional)

To simplify watchdog logic, the Jetson can send a dedicated heartbeat:

```
H\n
```

The STM32 resets its “last command” timer on any valid frame (including `M` and `L`); `H` can be used when no motion or lift change is needed.

---

## Summary table

| Direction   | Frame   | Meaning |
|------------|---------|---------|
| Jetson → STM32 | `M,w_fl,w_fr,w_rl,w_rr\n` | Set wheel speeds (rad/s). |
| Jetson → STM32 | `L,0\n` / `L,1\n`         | Lift down / up. |
| Jetson → STM32 | `H\n` (optional)          | Heartbeat. |
| STM32 → Jetson | `T,...\n` (optional)      | Telemetry. |

## Error handling

- **Jetson:** If serial write fails or the port is disconnected, log and optionally publish a diagnostic. Do not block the ROS node.
- **STM32:** On parse error (unknown prefix, bad number), ignore the line and optionally toggle an error LED. On receive timeout, stop motors and lift (watchdog).

## Binary variant (optional)

For higher throughput or smaller frames, a binary protocol can be used instead (e.g. fixed-size struct with sync byte and checksum). The ASCII format above is the reference for this repo; the Jetson bridge and STM32 skeleton can be extended to support a binary mode if needed.
