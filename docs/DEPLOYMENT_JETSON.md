# Deploying on Jetson Nano (real robot)

This document describes how to run the Mecanum AMR stack on the **Jetson Nano** with the **STM32** motor driver connected over UART. The same ROS 2 packages (Nav2, SLAM, task manager, etc.) are used; the hardware bridge replaces the simulation by forwarding `/cmd_vel` and lift commands to the STM32.

## Hardware setup

- **Jetson Nano** with Ubuntu 20.04 and ROS 2 Humble (or 22.04 if available for your image).
- **STM32** connected via UART:
  - On-board: Jetson 40-pin header UART → `/dev/ttyTHS1`.
  - USB–serial adapter: typically `/dev/ttyUSB0`.
- Shared **GND** between Jetson and STM32. 3.3 V or 5 V logic level as per your STM32; use a level shifter if needed.

See [../hardware/HARDWARE_ARCHITECTURE.md](../hardware/HARDWARE_ARCHITECTURE.md) and [../hardware/docs/PROTOCOL.md](../hardware/docs/PROTOCOL.md) for the protocol and wiring.

## Software on Jetson

### 1. ROS 2 and dependencies

Install ROS 2 Humble and the packages used by this repo (Nav2, SLAM Toolbox, cv_bridge, etc.). Clone this repository into your workspace and install dependencies:

```bash
cd ~/ros2_ws/src
git clone https://github.com/YOUR_USERNAME/mecanum_amr.git .
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Serial port access

Add your user to the `dialout` group so you can open the serial port without root:

```bash
sudo usermod -aG dialout $USER
```

Log out and back in (or reboot) for it to take effect.

### 3. pyserial (for the hardware bridge)

```bash
pip3 install pyserial
# or: sudo apt install python3-serial
```

### 4. Build

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Running the hardware bridge

Start the bridge so that all motion and lift commands from ROS are sent to the STM32:

```bash
# Default: /dev/ttyTHS1 @ 115200
ros2 launch mecanum_hw_bridge hw_bridge.launch.py

# USB–serial
ros2 launch mecanum_hw_bridge hw_bridge.launch.py serial_port:=/dev/ttyUSB0
```

Then start your autonomy stack (mapping, Nav2, task manager, etc.) as usual. The bridge subscribes to `/cmd_vel` and advertises `/lift/up` and `/lift/down`; when Nav2 or the task manager publish or call these, the commands are forwarded to the STM32.

## Watchdog

The bridge sends motion commands at a fixed rate (default 25 Hz). If no `/cmd_vel` is received for a configurable timeout (default 0.5 s), it sends **zero motion** (`M,0,0,0,0`) so the STM32 can stop the motors. Configure with parameters:

```bash
ros2 run mecanum_hw_bridge hw_bridge_node.py --ros-args \
  -p serial_port:=/dev/ttyTHS1 \
  -p cmd_timeout_sec:=0.5
```

## Optional: device rules for fixed name

If you use a USB–serial adapter and want a stable name (e.g. `/dev/ttyAMR`), add a udev rule and reload:

```bash
# Example: create /etc/udev/rules.d/99-amr-stm32.rules
# SUBSYSTEM=="tty", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="yyyy", SYMLINK+="ttyAMR"
sudo udevadm control --reload-rules
```

Then use `serial_port:=/dev/ttyAMR` in the launch.
