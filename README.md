# Mecanum AMR — Autonomous Mobile Robot

[![CI](https://github.com/Trkkhrmn/ros2_amr_mecanumbot/actions/workflows/ci.yml/badge.svg)](https://github.com/Trkkhrmn/ros2_amr_mecanumbot/actions/workflows/ci.yml)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo Classic](https://img.shields.io/badge/Gazebo-Classic%2011-333333?logo=open-in-new)](https://gazebosim.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

ROS 2 Humble + Gazebo simulation of a **mecanum-wheel** autonomous warehouse robot. It scans the environment with LiDAR, builds a map with SLAM, reads QR codes to get target poses, navigates there with Nav2, and uses a lift mechanism to pick and place loads.

---

## About this project

This repository contains the **simulation stack** and **hardware interface** for a Mecanum AMR developed as a team project at **Yıldız Technical University, Faculty of Electrical and Electronics**. The same architecture runs in **Gazebo** (this repo) and on the **physical robot** (Jetson Nano + STM32).

**Project team:**

| Role | Responsibility |
|------|----------------|
| **Tarık Kahraman** | Simulation (Gazebo, worlds, bringup, integration). |
| **Muhammed Sait Karadeniz** | Image processing, Jetson Nano software and integration. |
| **Samet Hasan Köse** | Motor control, STM32 firmware, motor drivers. |

This repo is the author's own work; the hardware protocol and bridge were designed for the real system. No code has been copied from other projects. The project was built around a **physical Mecanum AMR** (Jetson Nano + STM32 over UART); a lot of the autonomy was developed and tested in ROS 2 and Gazebo. This repository preserves that work: you get both a **working Gazebo simulation** (SLAM, Nav2, QR, lift, task state machine) and the **hardware integration layer** (UART protocol, Jetson bridge, STM32 parser and motor/lift stubs) for the real robot. More context: [docs/PROJECT_BACKGROUND.md](docs/PROJECT_BACKGROUND.md).

---

## Table of contents

- [Package layout](#-package-layout)
- [Hardware integration (real robot)](#-hardware-integration-real-robot)
- [Requirements](#-requirements)
- [Installation](#-installation)
- [Usage](#-usage)
- [Architecture](#-architecture)
- [Topics](#-topics)
- [License](#-license)

---

## Package layout

| Package | Description |
|---------|-------------|
| `mecanum_description` | URDF/Xacro robot model (LiDAR, camera, lift) |
| `mecanum_bringup` | Launch files to bring up the whole stack |
| `mecanum_navigation` | SLAM Toolbox + Nav2 config |
| `mecanum_control` | Lift control and mecanum kinematics (C++ and Python) |
| `mecanum_task_manager` | QR reading, task planning, state machine |
| `mecanum_simulation` | Gazebo warehouse world and spawn setup |
| `mecanum_hw_bridge` | Jetson ↔ STM32 UART bridge for real robot (cmd_vel + lift) |

---

## Hardware integration (real robot)

This repo was designed for a **physical Mecanum AMR**: **Jetson Nano** (ROS 2, Nav2, SLAM, QR, task manager) and **STM32** (motor drivers, lift). The hardware interface lives in the [`hardware/`](hardware/) folder. The same codebase covers **simulation** (Gazebo) and **real hardware** (Jetson + STM32 over UART).

### Stack overview

| Component | Role |
|-----------|------|
| **Jetson Nano** | Runs ROS 2 Humble: Nav2, SLAM, QR/task manager, kinematics. Publishes `/cmd_vel` and lift commands. Talks to STM32 over UART. |
| **STM32** | Receives motion and lift commands via UART. Drives 4× mecanum motors (PWM) and lift actuator. Watchdog stops motors if no command for ~300–500 ms. |

```
  Jetson Nano (ROS 2)  →  /cmd_vel, /lift/up|down  →  mecanum_hw_bridge  →  UART (115200)
                                                                                ↓
  STM32  →  parse protocol  →  PWM 4× wheels + lift
```

Connection: Jetson TX → STM32 RX, Jetson RX → STM32 TX (optional telemetry), GND common. Use `/dev/ttyTHS1` (Jetson 40-pin) or USB–serial `/dev/ttyUSB0`.

### UART protocol (Jetson ↔ STM32)

ASCII, line-based, 115200 8N1. Frames (LF-terminated):

| Direction | Frame | Meaning |
|-----------|--------|---------|
| Jetson → STM32 | `M,w_fl,w_fr,w_rl,w_rr\n` | Wheel speeds in rad/s (front_left, front_right, rear_left, rear_right). Sent at ~20–50 Hz. |
| Jetson → STM32 | `L,0\n` / `L,1\n` | Lift down / up. |
| Jetson → STM32 | `H\n` (optional) | Heartbeat. |
| STM32 → Jetson | `T,...\n` (optional) | Telemetry. |

The ROS package `mecanum_hw_bridge` subscribes to `/cmd_vel`, advertises `/lift/up` and `/lift/down`, and sends these frames to the STM32. Full protocol: [hardware/docs/PROTOCOL.md](hardware/docs/PROTOCOL.md). Block diagram and pins: [hardware/HARDWARE_ARCHITECTURE.md](hardware/HARDWARE_ARCHITECTURE.md).

### Running on Jetson (real robot)

1. Connect STM32 via UART; add user to `dialout`: `sudo usermod -aG dialout $USER` (then log out and back in).
2. Install pyserial: `pip3 install pyserial`.
3. Build workspace and run the bridge:

```bash
ros2 launch mecanum_hw_bridge real_robot_bringup.launch.py serial_port:=/dev/ttyTHS1
```

(Use `serial_port:=/dev/ttyUSB0` for USB–serial.) The bridge sends motion at a fixed rate; if no `/cmd_vel` for the timeout (e.g. 0.5 s), it sends zero motion so the STM32 can stop the motors. Details: [docs/DEPLOYMENT_JETSON.md](docs/DEPLOYMENT_JETSON.md).

### Quick start on your own mecanum robot

You need: 4 mecanum wheels (+ drivers), optional lift; Jetson or any Linux with ROS 2 Humble; STM32 (or other MCU) with one UART to the Jetson.

- **STM32:** Copy `hardware/stm32/` protocol code into your project; implement `motor_set_speeds(w_fl,w_fr,w_rl,w_rr)` and `lift_set(0|1)` with your PWM/HAL. UART 115200 8N1; feed bytes into the parser and run a watchdog (stop motors if no frame for 500 ms). Flash the board.
- **Jetson:** Clone repo, `rosdep install`, `colcon build`, then run the bridge with the correct `serial_port`. Tune `wheel_radius`, `lx`, `ly` in `mecanum_hw_bridge` config if your robot dimensions differ.
- **First test:** With bridge and STM32 connected, publish once to `/cmd_vel` (e.g. `ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"`) and call `/lift/up` and `/lift/down` to verify motion and lift. Then run the full stack (SLAM, Nav2) as in simulation.

Step-by-step wiring, flash, and troubleshooting: [docs/REAL_ROBOT_QUICKSTART.md](docs/REAL_ROBOT_QUICKSTART.md) and [hardware/README.md](hardware/README.md).

**Real system — overview and tests:**

| ![Real robot overview](docs/images/robot_overview.png) | ![Lift system](docs/images/lift_system.png) |
|:---:|:---:|
| Physical AMR (Jetson Nano, LiDAR, enclosure). | AMR moving under the load and lifting it. |

![Localization and path planning test at Yıldız Technical University](docs/images/localization.png)

*Obstacle avoidance and path planning tests — Yıldız Technical University, Faculty of Electrical and Electronics.*

---

## Requirements

- **ROS 2 Humble** (Ubuntu 22.04 recommended)
- **Gazebo Classic 11**
- **Nav2**, **SLAM Toolbox**, **cv_bridge**

For lift motion in Gazebo you also need:

```bash
sudo apt install ros-humble-gazebo-ros2-control ros-humble-ros2-control \
  ros-humble-ros2-controllers ros-humble-ros2controlcli
```

---

## Installation

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Trkkhrmn/ros2_amr_mecanumbot.git .

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Docker

```bash
docker build -f docker/Dockerfile -t mecanum_amr .
docker run -it --rm \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  mecanum_amr \
  ros2 launch mecanum_bringup sim_bringup.launch.py
```

---

## Usage

### Starting the simulation

Default run: warehouse world with robot and lift. Optional: Nav2 + SLAM, or minimal/empty world.

![Simulation environment — warehouse with shelves and QR stations](docs/images/general_map.png)

```bash
# Warehouse world + robot + lift (default)
ros2 launch mecanum_bringup sim_bringup.launch.py

# With Nav2 + SLAM
ros2 launch mecanum_bringup sim_bringup.launch.py launch_navigation:=true

# Gazebo + robot only (minimal or empty world)
ros2 launch mecanum_bringup sim_bringup.launch.py world:=minimal

# Robot model only (RViz)
ros2 launch mecanum_description display.launch.py

# Navigation only (SLAM mode)
ros2 launch mecanum_navigation navigation.launch.py slam_mode:=true
```

**Worlds:** Default is `warehouse` (shelves, QR stations). You can use `world:=minimal` or `world:=empty`.

**Task manager (QR + Nav2):** Use `launch_task_manager:=true`; Nav2 must be installed (`ros-humble-nav2-bringup`, `ros-humble-nav2-msgs`).

### Green lane (camera view + follow)

In the warehouse world you can open a **separate window** with the robot's camera view and OpenCV indicators (green strip detection, path centering). Optionally, the robot can **follow the two green lines** automatically. The default world `warehouse` includes the green lines.

![Green line following in the warehouse](docs/images/Green_line.gif)

1. **Start the simulation** (if not already running):  
   `ros2 launch mecanum_bringup sim_bringup.launch.py`

2. **Camera window + indicators only** (no motion): in a second terminal,  
   `ros2 run mecanum_control green_lane_detector.py`  
   A window shows the robot camera; OpenCV draws **GREEN DETECTED**, **CENTERED**, or **OFFSET LEFT/RIGHT**, with a yellow line for strip center and blue for image center.

3. **Follow the two green lines** (robot moves):  
   `ros2 run mecanum_control green_lane_detector.py --follow`  
   The node publishes `/cmd_vel` to keep the robot between the two green lines; if no green is detected, it stops.

Requires `cv_bridge` and OpenCV (e.g. `sudo apt install ros-humble-cv-bridge`). Topic: `/camera/image_raw`. Full steps: [docs/GREEN_LANE.md](docs/GREEN_LANE.md).

### SLAM mapping (warehouse world)

Open the warehouse world, SLAM Toolbox, and RViz in one go:

```bash
ros2 launch mecanum_navigation test_mapping_slam.launch.py world:=warehouse
```

In a second terminal, drive the robot with teleop so the map fills in:

```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use **w/s** for forward/back, **a/d** for turn. When the map is complete, save it to the package `maps/` folder (from repo root, with launch and teleop still running):

```bash
ros2 run nav2_map_server map_saver_cli -f src/mecanum_navigation/maps/warehouse_map --ros-args -p map_subscribe_transient_local:=true
```

This creates `warehouse_map.yaml` and `warehouse_map.pgm` under `src/mecanum_navigation/maps/`. Full guide: [docs/MAPPING_WAREHOUSE.md](docs/MAPPING_WAREHOUSE.md).

**QR-based navigation** — robot moving to a target pose read from a QR code:

![QR to goal](docs/images/mapping.png)

**Occupancy grid** built from LiDAR (manual mapping run):

![Warehouse map from LiDAR](docs/images/warehouse_map.png)

### Troubleshooting

| Issue | What to do |
|-------|------------|
| "Entity [mecanum_amr] already exists" | Close all Gazebo windows and run the launch again. |
| `gzserver` exit 255 | Run `pkill -9 gzserver; pkill -9 gzclient`, then try again. For more detail: `gzserver --verbose $(ros2 pkg prefix mecanum_simulation)/share/mecanum_simulation/worlds/warehouse.world`. If you have GPU issues, try `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch ...`. |

---

## Architecture

![ROS 2 architecture](docs/images/ros2_architecture.png)

**State machine:**  
`IDLE → READING_QR → NAVIGATING_TO_LOAD → ALIGNING → LIFTING → NAVIGATING_TO_DROP → LOWERING → RETURNING_HOME → IDLE`

---

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `LaserScan` | LiDAR data |
| `/cmd_vel` | `Twist` | Velocity command |
| `/odom` | `Odometry` | Odometry |
| `/map` | `OccupancyGrid` | SLAM map |
| `/qr/data` | `String` | Raw QR payload |
| `/qr/goal_pose` | `PoseStamped` | Parsed goal from QR |
| `/robot_state` | `String` | Current state machine state |

**QR format:** `x:<float>;y:<float>;yaw:<float>` (e.g. `x:3.5;y:1.2;yaw:1.57`)

---

## Media files (for maintainers)

Place the following in `docs/images/` so the README displays them correctly:

| File | Description |
|------|-------------|
| `robot_overview.png` | General view of the real robot |
| `lift_system.png` | AMR lifting the load (real system) |
| `localization.png` | Obstacle avoidance & path planning test (Yıldız Teknik Üniversitesi) |
| `mapping.png` | QR-based navigation to goal (simulation) |
| `warehouse_map.png` | LiDAR-based occupancy map |
| `general_map.png` | General view of the simulation environment |
| `ros2_architecture.png` | ROS 2 architecture diagram |
| `Green_line.gif` | Green line following in warehouse (camera + OpenCV) |

---

## License

[MIT License](LICENSE). Contributions are welcome: fork, branch, make your changes, then open a PR. Use ROS 2 Humble on Ubuntu 22.04 and `colcon build --symlink-install`; CI runs on push to `main`/`develop` and on PRs. Full guide (code style, tests, maintainer): [CONTRIBUTING.md](CONTRIBUTING.md).
