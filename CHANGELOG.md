# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [1.0.0] — 2025-02-23

### Added

- ROS 2 Humble workspace: simulation (Gazebo), Nav2, SLAM Toolbox, QR-based task manager, lift control.
- Packages: `mecanum_description`, `mecanum_bringup`, `mecanum_navigation`, `mecanum_control`, `mecanum_task_manager`, `mecanum_simulation`.
- **Hardware integration**
  - Jetson Nano ↔ STM32 UART protocol (ASCII: motion `M,...`, lift `L,0`/`L,1`).
  - `mecanum_hw_bridge` package: `/cmd_vel` and lift services forwarded to STM32; config via YAML; kinematics unit tests.
  - STM32 firmware interface: protocol parser (C), motor/lift stubs, host test (Makefile + `main_host`).
- Docs: hardware architecture, protocol spec, Jetson deployment, project background.
- CI: build, colcon test, HW bridge kinematics test, STM32 protocol host test.
- Repo polish: `.editorconfig`, `.clang-format`, LICENSE (MIT), CONTRIBUTING, SECURITY.

### Notes

- Replace `YOUR_USERNAME` in README and CI badge with your GitHub username after pushing.

[1.0.0]: https://github.com/YOUR_USERNAME/mecanum_amr/releases/tag/v1.0.0
