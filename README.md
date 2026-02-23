# Mecanum AMR — Autonomous Mobile Robot

[![CI](https://github.com/Trkkhrmn/ros2_amr_mecanumbot/actions/workflows/ci.yml/badge.svg)](https://github.com/Trkkhrmn/ros2_amr_mecanumbot/actions/workflows/ci.yml)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo Classic](https://img.shields.io/badge/Gazebo-Classic%2011-333333?logo=open-in-new)](https://gazebosim.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

ROS 2 Humble + Gazebo sim of a **mecanum-wheel** autonomous warehouse robot: LiDAR for scanning, SLAM for mapping, QR codes for target poses, Nav2 for navigation, and a lift for pick-and-place.

---

## About this project

This repo has the **simulation** and **hardware side** of a Mecanum AMR we built as a team project at **Yıldız Technical University, Faculty of Electrical and Electronics**. Same stuff runs in **Gazebo** (here) and on the **real robot** (Jetson Nano + STM32).

**Project background (why this repo exists):**

The whole thing started from a **real physical Mecanum AMR** — big mecanum robot with a lift for warehouse-style pick-and-place. We built it with:

- **Jetson Nano** as the brain (ROS 2, Nav2, SLAM, QR goals, task logic).
- **STM32** for the low-level stuff (four mecanum motors + lift).
- **UART** between Jetson and STM32 (protocol and bridge code are in this repo under `hardware/` and the `mecanum_hw_bridge` package).

Most of the autonomous software was developed and tested in **ROS 2 and Gazebo**. On the real robot we only ran it manually (remote control + manual lift) for demos and a fair — we never had the full autonomous stack running on the real hardware. After the team split, the original project and code were no longer around.

So this repo is here to **keep that work and share it**:

1. **Simulation** — Same architecture (SLAM, Nav2, QR, lift, task state machine) is implemented and works in Gazebo.
2. **Hardware integration** — The Jetson–STM32 link is documented and coded: UART protocol, ROS 2 bridge on the Jetson, and on the STM32 side the protocol parser plus stubs for motors and lift. So the design is clearly for a real system (Jetson + STM32), not just sim.

Bottom line: you get both a **working Gazebo sim** and the **hardware layer** that was meant for the actual robot. All of this is our own work; the protocol and bridge were designed for our system and we didn’t copy from other projects.

---

## Table of contents

- [Package layout](#-package-layout)
- [Hardware integration (real robot)](#-hardware-integration-real-robot)
- [Requirements](#-requirements)
- [Installation](#-installation)
- [Usage](#-usage)
- [Architecture](#-architecture)
- [Topics](#-topics)
- [Project team](#-project-team)
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

This repo was built for a **real Mecanum AMR**: **Jetson Nano** (ROS 2, Nav2, SLAM, QR, task manager) and **STM32** (motor drivers, lift). All the hardware-related docs and code are in [`hardware/`](hardware/). Same codebase works for **sim** (Gazebo) and **real robot** (Jetson + STM32 over UART).

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

The ROS package `mecanum_hw_bridge` subscribes to `/cmd_vel`, advertises `/lift/up` and `/lift/down`, and sends these frames to the STM32. Full protocol details: [hardware/docs/PROTOCOL.md](hardware/docs/PROTOCOL.md).

### Hardware architecture (block diagram and roles)

This is the physical setup the repo was designed for: a mecanum AMR with lift, **Jetson Nano** as main computer and **STM32** as motor-driver and I/O.

**Block diagram:**

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
                    │  PWM: 4x wheels, 1x lift                           │
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

**Jetson Nano:** Runs the full autonomy (mapping, localization, Nav2, QR, task state machine). It outputs `/cmd_vel` and lift service calls; the hardware bridge turns those into UART frames for the STM32. Serial to STM32: `/dev/ttyTHS1` (40-pin header) or USB-serial `/dev/ttyUSB0`. Use 3.3V or a level shifter if your STM32 is 3.3V. LiDAR and camera are on the Jetson; ROS drivers publish `/scan` and `/camera/image_raw`.

**STM32:** Only does low-level. It receives motion and lift commands and drives the motors and lift. You can add encoder feedback or telemetry back to the Jetson if you want. Safety: **watchdog** — if no valid command for 300–500 ms, it should stop all motors and lift.

**Connection summary:**

| From      | To    | Interface |
|-----------|-------|-----------|
| Jetson TX | STM32 RX | UART (commands) |
| Jetson RX | STM32 TX | UART (optional telemetry) |
| GND       | GND   | Common ground |
| STM32     | 4x motors | PWM / motor driver |
| STM32     | Lift  | GPIO/PWM |

Pinout and timer channels depend on your board; see [hardware/stm32/](hardware/stm32/) for the protocol code and placeholders.

### Running on Jetson (real robot)

1. Connect the STM32 over UART. Add yourself to `dialout` so you can use the serial port: `sudo usermod -aG dialout $USER` (then log out and back in).
2. Install pyserial: `pip3 install pyserial`.
3. Build the workspace and run the bridge:

```bash
ros2 launch mecanum_hw_bridge real_robot_bringup.launch.py serial_port:=/dev/ttyTHS1
```

(Use `serial_port:=/dev/ttyUSB0` if you’re on USB–serial.) The bridge keeps sending motion at a fixed rate; if there’s no `/cmd_vel` for a while (e.g. 0.5 s), it sends zero motion so the STM32 stops the motors. More detail: [docs/DEPLOYMENT_JETSON.md](docs/DEPLOYMENT_JETSON.md).

### Quick start on your own mecanum robot

You’ll need: 4 mecanum wheels (+ drivers), optional lift; a Jetson or any Linux box with ROS 2 Humble; and an STM32 (or similar) with one UART to the Jetson.

- **STM32:** Drop the protocol code from `hardware/stm32/` into your project and implement `motor_set_speeds(w_fl,w_fr,w_rl,w_rr)` and `lift_set(0|1)` with your PWM/HAL. UART 115200 8N1; feed incoming bytes into the parser and run a watchdog (stop motors if no frame for 500 ms). Then flash the board.
- **Jetson:** Clone the repo, `rosdep install`, `colcon build`, then run the bridge with the right `serial_port`. If your robot has different wheel size or axle spacing, tweak `wheel_radius`, `lx`, `ly` in the `mecanum_hw_bridge` config.
- **First test:** With the bridge and STM32 connected, publish once to `/cmd_vel` and try `/lift/up` and `/lift/down` to see if motion and lift work. After that you can run the full stack (SLAM, Nav2) like in sim.

Step-by-step wiring, flashing, and troubleshooting: [docs/REAL_ROBOT_QUICKSTART.md](docs/REAL_ROBOT_QUICKSTART.md) and [hardware/README.md](hardware/README.md).

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

By default you get the warehouse world with the robot and lift. You can also turn on Nav2 + SLAM or use a minimal/empty world.

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

**Worlds:** Default is `warehouse` (shelves, QR stations). Or use `world:=minimal` / `world:=empty`.

**Task manager (QR + Nav2):** Set `launch_task_manager:=true`. You’ll need Nav2 installed (`ros-humble-nav2-bringup`, `ros-humble-nav2-msgs`).

### Green lane (camera view + follow)

In the warehouse world you can open a **separate window** with the robot’s camera and OpenCV overlays (green strip detection, path centering). You can also let the robot **follow the two green lines** on its own. The default `warehouse` world has those lines.

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

Launch the warehouse world + SLAM Toolbox + RViz in one go:

```bash
ros2 launch mecanum_navigation test_mapping_slam.launch.py world:=warehouse
```

In another terminal, drive the robot with teleop so the map builds up:

```bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use **w/s** for forward/back and **a/d** to turn. When you’re happy with the map, save it (from repo root, with the launch and teleop still running):

```bash
ros2 run nav2_map_server map_saver_cli -f src/mecanum_navigation/maps/warehouse_map --ros-args -p map_subscribe_transient_local:=true
```

That creates `warehouse_map.yaml` and `warehouse_map.pgm` under `src/mecanum_navigation/maps/`. Full guide: [docs/MAPPING_WAREHOUSE.md](docs/MAPPING_WAREHOUSE.md).

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

## Project team

| Role | Who | What they did |
|------|-----|----------------|
| Simulation | **Tarık Kahraman** | Gazebo, worlds, bringup, integration |
| Image processing & Jetson | **Muhammed Sait Karadeniz** | Camera stuff, Jetson software and integration |
| Motors & firmware | **Samet Hasan Köse** | Motor control, STM32 firmware, motor drivers |

---

## License

[MIT License](LICENSE). If you want to contribute: fork, make your changes, open a PR. We use ROS 2 Humble on Ubuntu 22.04 and `colcon build --symlink-install`; CI runs on pushes to `main`/`develop` and on PRs. For code style and tests see [CONTRIBUTING.md](CONTRIBUTING.md).
