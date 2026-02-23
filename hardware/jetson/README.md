# Jetson Nano side — ROS 2 hardware bridge

The ROS 2 node that runs on the **Jetson Nano** and talks to the STM32 over UART lives in the main repo as a colcon package:

- **Package:** `mecanum_hw_bridge` (under `src/mecanum_hw_bridge/`)
- **Node:** `hw_bridge_node.py` — subscribes to `/cmd_vel`, provides `/lift/up` and `/lift/down` services, and sends ASCII frames to the STM32.

## Dependencies

- **pyserial:** `pip install pyserial` (or `sudo apt install python3-serial` on Ubuntu/Jetson).

## Build and run

From the workspace root (after `colcon build` and `source install/setup.bash`):

```bash
# Default port (Jetson 40-pin UART)
ros2 launch mecanum_hw_bridge hw_bridge.launch.py

# USB–serial adapter
ros2 launch mecanum_hw_bridge hw_bridge.launch.py serial_port:=/dev/ttyUSB0
```

Or run the node directly with parameters:

```bash
ros2 run mecanum_hw_bridge hw_bridge_node.py --ros-args \
  -p serial_port:=/dev/ttyTHS1 -p baud_rate:=115200
```

## Parameters

| Parameter           | Default      | Description |
|---------------------|-------------|-------------|
| `serial_port`       | `/dev/ttyTHS1` | UART device. |
| `baud_rate`         | `115200`    | Baud rate. |
| `wheel_radius`      | `0.075`     | Wheel radius (m), for inverse kinematics. |
| `lx`, `ly`          | `0.25`, `0.35` | Half front–rear and left–right (m). |
| `max_wheel_speed`   | `15.0`      | Max wheel speed (rad/s). |
| `cmd_timeout_sec`   | `0.5`       | If no `/cmd_vel` for this long, send zero motion (watchdog). |
| `motion_rate_hz`    | `25.0`      | Rate at which motion commands are sent to STM32. |

## Protocol

Sent to STM32 (see [../docs/PROTOCOL.md](../docs/PROTOCOL.md)):

- **Motion:** `M,w_fl,w_fr,w_rl,w_rr\n` (rad/s), at `motion_rate_hz`.
- **Lift up:** `L,1\n` (on `/lift/up` service call).
- **Lift down:** `L,0\n` (on `/lift/down` service call).

When no `/cmd_vel` is received for `cmd_timeout_sec`, the node sends `M,0,0,0,0\n` so the STM32 can implement a software watchdog and stop the motors.
