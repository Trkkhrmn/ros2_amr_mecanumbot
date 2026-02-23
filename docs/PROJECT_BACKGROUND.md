# Project background — real robot and this repo

This repository reflects a **physical Mecanum AMR** project developed by a team: a large mecanum-wheel autonomous mobile robot with a lift, designed for warehouse-style pick-and-place. The robot was built with:

- **Jetson Nano** as the main computer (ROS 2, Nav2, SLAM, QR-based goals, task logic).
- **STM32** as the motor-driver MCU (four mecanum motors and lift actuator).
- Communication between Jetson and STM32 over **UART** (protocol and integration code are in this repo under `hardware/` and the `mecanum_hw_bridge` package).

A lot of autonomous software was written and tested in **ROS 2 and Gazebo** during development. On the real platform, the robot was operated **manually** (remote control and manual positioning for lift pick/place) for demos and a fair; the full autonomous stack was never run on the physical robot. After the team dissolved, access to the original project and code was lost.

This repo exists to **preserve and share** that work:

1. **Simulation** — The same architecture (SLAM, Nav2, QR, lift, task state machine) is implemented and runnable in Gazebo.
2. **Hardware integration** — The Jetson–STM32 link is documented and implemented: UART protocol, ROS 2 bridge on the Jetson, and STM32-side protocol parsing and motor/lift stubs. That way, the design is clearly tied to a real system (Jetson Nano + STM32), not only to simulation.

So the codebase is both a **working Gazebo simulation** and a **real-hardware integration layer** that was designed for the actual robot.
