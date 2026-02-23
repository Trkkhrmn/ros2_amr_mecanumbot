# AMCL + Nav2 test with a saved map

## 1. Where to put the map files

- `warehouse_map.yaml` and `warehouse_map.pgm` (or `.png`) must be **in the same folder**.
- The `image: warehouse_map.pgm` line in the YAML must match the filename you use (pgm or png).
- Put them in the package `maps/` folder:
  - `src/mecanum_navigation/maps/warehouse_map.yaml`
  - `src/mecanum_navigation/maps/warehouse_map.pgm`

Then after `colcon build` they get copied to the install path and the default map path works.

## 2. Build

```bash
cd ~/mecanum_amr
colcon build --packages-select mecanum_navigation --allow-overriding mecanum_navigation
source install/setup.bash
```

## 3. AMCL + Nav2 launch

**Default map** (from install `maps/warehouse_map.yaml`):

```bash
ros2 launch mecanum_navigation test_mapping_nav.launch.py
```

**Custom map path** (e.g. source maps):

```bash
ros2 launch mecanum_navigation test_mapping_nav.launch.py \
  map_yaml:=/path/to/mecanum_navigation/maps/warehouse_map.yaml
```

This launch:
- Starts Gazebo with `test_mapping.world`
- Spawns the robot
- **map_server** publishes the saved map on `/map`
- **AMCL** localizes using map + LiDAR + odom
- **Nav2** (planner + controller) drives to the goal
- RViz opens with `navigation.rviz` (map, costmaps, AMCL particles, paths)

## 4. Testing in RViz

1. **Initial pose (2D Pose Estimate)**  
   Click "2D Pose Estimate", drag on the map to where the robot **actually is** (often the yellow square / origin), set orientation with the green arrow. AMCL particles (orange arrows) should cluster there.

2. **Optional: a few seconds of teleop**  
   Run `ros2 run teleop_twist_keyboard teleop_twist_keyboard` and move the robot a bit; AMCL converges faster.

3. **Set goal (Nav2 Goal)**  
   Click "Nav2 Goal", click on the map for the target and set orientation. The robot should follow the planned path.

4. **Check**  
   - Global path (green): planner path  
   - Local path (orange): controller path  
   - If AMCL particles are in one spot, localization is good.

## 5. Troubleshooting

- **Map not showing:** Is `map_yaml` correct? Is the file in `image:` in the same folder?
- **AMCL scattered / robot drifting on map:** Set 2D Pose Estimate again at the real robot pose; move a bit with teleop.
- **"Goal aborted" / no path:** Is the goal in an obstacle or in costmap? Try a more open point.
