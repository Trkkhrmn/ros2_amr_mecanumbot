# Mecanum AMR — Autonomous Mobile Robot

[![CI](https://github.com/YOUR_USERNAME/mecanum_amr/actions/workflows/ci.yml/badge.svg)](https://github.com/YOUR_USERNAME/mecanum_amr/actions/workflows/ci.yml)
[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![Gazebo Classic](https://img.shields.io/badge/Gazebo-Classic%2011-333333?logo=open-in-new)](https://gazebosim.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

ROS 2 Humble + Gazebo simulation of a **mecanum-wheel** autonomous warehouse robot. It scans the environment with LiDAR, builds a map with SLAM, reads QR codes to get target poses, navigates there with Nav2, and uses a lift mechanism to pick and place loads.

| [![Real robot](docs/real_robot/robot_photo.jpg)](docs/real_robot/) | [![Simulation](docs/images/sim_screenshot.png)](docs/images/) |
|:---:|:---:|
| *Real robot (optional: add your photo)* | *Simulation: Gazebo or RViz (optional)* |

*(Replace the paths above with your images, or remove the table until you add them. Suggested: one real-robot photo, one sim screenshot — see [Media](#-media).)*

---

## About this project

**Author:** Tarik Kahraman  

This repository is my own work. It mirrors a real Mecanum AMR project (Jetson Nano + STM32) built with a team; the hardware interface and protocol in this repo were designed for that system. See [docs/PROJECT_BACKGROUND.md](docs/PROJECT_BACKGROUND.md) for context. No code has been copied from other projects or tutorials, and I did not use AI to write the code—only for minor things like comments and documentation. The design and implementation are original.

---

## Table of contents

- [Package layout](#-package-layout)
- [Hardware integration (real robot)](#-hardware-integration-real-robot)
- [Requirements](#-requirements)
- [Installation](#-installation)
- [Usage](#-usage)
- [Architecture](#-architecture)
- [Topics](#-topics)
- [Media](#-media)
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

This repo was designed for a **physical Mecanum AMR**: **Jetson Nano** (ROS 2, Nav2, SLAM, QR, task manager) and **STM32** (motor drivers, lift). The hardware interface is in the [`hardware/`](hardware/) folder:

| Item | Description |
|------|-------------|
| [hardware/README.md](hardware/README.md) | Stack overview (Jetson + STM32), quick start. |
| [hardware/HARDWARE_ARCHITECTURE.md](hardware/HARDWARE_ARCHITECTURE.md) | Block diagram, connections, roles. |
| [hardware/docs/PROTOCOL.md](hardware/docs/PROTOCOL.md) | UART protocol: `M,w_fl,w_fr,w_rl,w_rr` (motion), `L,0` / `L,1` (lift). |
| **Jetson** | ROS 2 package `mecanum_hw_bridge`: subscribes to `/cmd_vel`, provides `/lift/up` and `/lift/down`, sends ASCII frames to STM32. |
| **STM32** | [hardware/stm32/](hardware/stm32/): protocol parser (C) and stubs for motor/lift control; integrate with your HAL and PWM. |
| [docs/DEPLOYMENT_JETSON.md](docs/DEPLOYMENT_JETSON.md) | How to run on Jetson Nano, serial port, watchdog. |

So the same codebase covers **simulation** (Gazebo) and **real hardware** (Jetson + STM32 over UART).

**Want to run this on your own mecanum robot?** → [Real robot quick start](docs/REAL_ROBOT_QUICKSTART.md) (what to wire, what to flash, how to test).

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
git clone https://github.com/YOUR_USERNAME/mecanum_amr.git .

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Replace `YOUR_USERNAME` with your GitHub username after you push the repo.

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

### Troubleshooting

| Issue | What to do |
|-------|------------|
| "Entity [mecanum_amr] already exists" | Close all Gazebo windows and run the launch again. |
| `gzserver` exit 255 | Run `pkill -9 gzserver; pkill -9 gzclient`, then try again. For more detail: `gzserver --verbose $(ros2 pkg prefix mecanum_simulation)/share/mecanum_simulation/worlds/warehouse.world`. If you have GPU issues, try `LIBGL_ALWAYS_SOFTWARE=1 ros2 launch ...`. |

---

## Architecture

```
┌─────────────────────────────────────────────────┐
│                 Gazebo Simulation                │
│  ┌──────────┐  ┌──────────┐  ┌───────────────┐  │
│  │  LiDAR   │  │  Camera  │  │  Planar Move   │  │
│  └────┬─────┘  └────┬─────┘  └──────┬────────┘  │
└───────┼─────────────┼───────────────┼────────────┘
        │ /scan       │ /camera/image  │ /odom
        ▼             ▼               │
   ┌─────────┐   ┌──────────┐        │
   │  SLAM   │   │ QR Reader│        │
   │Toolbox  │   └────┬─────┘        │
   └────┬────┘        │ /qr/goal_pose│
        │ /map        ▼              │
        │       ┌──────────────┐     │
        │       │ Task Manager │     │
        │       │ State Machine│     │
        │       └──────┬───────┘     │
        │              │ NavigateToPose
        ▼              ▼
   ┌─────────────────────┐
   │        Nav2         │
   │ (Planner+Controller)│──── /cmd_vel ──► Robot
   └─────────────────────┘
```

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

## Media

**Hangi görseller olmalı?** (1) Gerçek robot fotoğrafı → `docs/real_robot/robot_photo.jpg` (araç veya kaldıraç; README başında gösterebilirsin). (2) Simülasyon ekran görüntüsü → `docs/images/sim_screenshot.png` (Gazebo veya RViz). (3–4) İsteğe bağlı: mimari diyagram, video linki. Görselleri ekleyince en üstteki tablodaki yolları güncelle veya tabloyu kaldır.


---

## License

[MIT License](LICENSE). See [CONTRIBUTING.md](CONTRIBUTING.md) if you want to contribute.
