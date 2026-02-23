# Running on a real mecanum robot

If you have a mecanum-wheel robot (4 motors, optional lift) and want to try this repo on it, this page is the straight path: what you need, what to do on the STM32, what to do on the Jetson, and how to test.

No fluff — just enough so someone can clone, wire, flash, and run.

---

## What you need

- **Robot:** 4 mecanum wheels (with motor drivers), optional lift. Any layout is fine as long as you know wheel radius and axle half-distances (lx, ly) for kinematics.
- **Jetson Nano** (or any Linux PC with ROS 2 Humble) to run the ROS 2 stack and the hardware bridge.
- **STM32** (or another MCU) that talks to the motor drivers and lift. It only needs one UART to the Jetson.
- **Connection:** UART between Jetson and STM32 (TX→RX, RX→TX, GND common). 115200 baud. Use the Jetson 40-pin UART or a USB–serial adapter.

---

## 1. STM32 side

The repo has the protocol parser and stubs; you add your motor and lift control.

1. **Copy the protocol code** from `hardware/stm32/Core/` into your STM32 project (CubeIDE, Make, etc.):
   - `protocol.c`, `protocol.h`
   - `motor_lift_stub.c` — **replace the stubs** with your real code: in `motor_set_speeds(w_fl, w_fr, w_rl, w_rr)` set PWM (or velocity setpoints) for the four wheels; in `lift_set(0)` / `lift_set(1)` drive the lift down/up.

2. **UART:** Configure 115200 8N1. In your UART RX callback (or idle interrupt), for each received byte call:
   - `protocol_feed_byte(byte);`
   - `protocol_set_last_rx_tick(your_tick_ms);`  
   so the watchdog knows data is arriving.

3. **Watchdog:** In a timer or main loop, if `get_tick_ms() - protocol_last_rx_tick() > 500` (or your chosen timeout), call `motor_set_speeds(0, 0, 0, 0)` and stop the lift so the robot doesn’t keep moving if the Jetson stops sending.

4. Build and flash as you usually do. The Jetson will send lines like `M,1.2,-0.5,0.8,-0.3\n` (wheel speeds in rad/s) and `L,0\n` / `L,1\n` for lift; the parser in `protocol.c` already handles these.

Details and host test: see [hardware/stm32/README.md](../hardware/stm32/README.md) and [hardware/docs/PROTOCOL.md](../hardware/docs/PROTOCOL.md).

---

## 2. Jetson (or Linux + ROS 2) side

1. **Clone and build:**
   ```bash
   cd ~/ros2_ws/src
   git clone https://github.com/YOUR_USERNAME/mecanum_amr.git .
   cd ~/ros2_ws
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --symlink-install
   source install/setup.bash
   ```

2. **Serial port:** Connect the STM32. If you use a USB–serial adapter:
   ```bash
   sudo usermod -aG dialout $USER
   ```
   Log out and back in. Check the port: `ls /dev/ttyUSB*` or `ls /dev/ttyTHS*` (Jetson built-in).

3. **pyserial:**
   ```bash
   pip3 install pyserial
   ```

4. **Run the bridge** (this forwards `/cmd_vel` and lift services to the STM32):
   ```bash
   ros2 launch mecanum_hw_bridge real_robot_bringup.launch.py serial_port:=/dev/ttyUSB0
   ```
   Use your actual port (e.g. `/dev/ttyTHS1` on Jetson 40-pin).

5. **Tune kinematics if needed:** If your robot has different wheel size or axle spacing, edit `src/mecanum_hw_bridge/config/hw_bridge.yaml` (wheel_radius, lx, ly) or override with parameters when you run the node.

---

## 3. First test

With the bridge running and the STM32 flashed and connected:

- **Motion:** In another terminal:
  ```bash
  source install/setup.bash
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  ```
  The robot should move forward a bit (or the wheels should turn). If nothing happens, check wiring, UART port, and that `motor_set_speeds` is actually driving your motors.

- **Lift (if you have it):**
  ```bash
  ros2 service call /lift/up std_srvs/srv/Trigger {}
  ros2 service call /lift/down std_srvs/srv/Trigger {}
  ```

- **Teleop (optional):** Install and run:
  ```bash
  sudo apt install ros-humble-teleop-twist-keyboard
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
  ```
  So you can drive the robot from the keyboard while the bridge is running.

After this works, you can run the full stack (SLAM, Nav2, task manager) the same way as in simulation; the only difference is that the bridge sends commands to the real STM32 instead of Gazebo.

---

## If something doesn’t work

- **Port not found:** `ls /dev/tty*` and try the right one. On Jetson, built-in UART is often `/dev/ttyTHS1`.
- **Robot doesn’t move:** Confirm the STM32 is receiving data (e.g. blink an LED in `motor_set_speeds` or log over another UART). Confirm `motor_set_speeds` is connected to your PWM/drivers.
- **Wrong direction or weird motion:** Check wheel order (front_left, front_right, rear_left, rear_right) and signs in your `motor_set_speeds` (which way is “positive” for each motor).
- **Bridge crashes or “Serial open failed”:** Check permissions (`dialout`), that nothing else is using the port (e.g. another terminal or an IDE), and baud rate (115200).

More detail: [DEPLOYMENT_JETSON.md](DEPLOYMENT_JETSON.md), [hardware/README.md](../hardware/README.md).
