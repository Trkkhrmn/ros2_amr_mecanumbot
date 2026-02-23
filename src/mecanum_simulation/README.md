# mecanum_simulation

## World options

- `warehouse` — Custom warehouse.world (default)
- `minimal`, `empty` — Simple worlds
- **`small_warehouse`** — AWS RoboMaker Small Warehouse (shelves, floor, walls, etc.)

## Using AWS Small Warehouse

You can use the AWS world in two ways:

### 1) From source

If you cloned the AWS repo into your workspace (e.g. `ros2_ws/src/aws-robomaker-small-warehouse-world`), set the **repo root** with an env var:

```bash
export AWS_WAREHOUSE_ROOT=/path/to/aws-robomaker-small-warehouse-world
ros2 launch mecanum_bringup sim_bringup.launch.py world:=small_warehouse
```

`GAZEBO_MODEL_PATH` is extended with `$AWS_WAREHOUSE_ROOT/models`; the world file is `$AWS_WAREHOUSE_ROOT/worlds/small_warehouse/small_warehouse.world`.

### 2) From installed package

If you built `aws_robomaker_small_warehouse_world` and did `source install/setup.bash`, no extra env is needed:

```bash
ros2 launch mecanum_bringup sim_bringup.launch.py world:=small_warehouse
```

## QR station textures

QR models (`qr_station_loading`, `qr_station_transfer`, `qr_station_unloading`) use PNG textures. Put the PNGs here:

- `src/mecanum_simulation/models/qr_station_loading/materials/textures/qr_station_loading.png`
- `src/mecanum_simulation/models/qr_station_transfer/materials/textures/qr_station_transfer.png`
- `src/mecanum_simulation/models/qr_station_unloading/materials/textures/qr_station_unloading.png`

Then run `colcon build --packages-select mecanum_simulation`; textures are copied to install and Gazebo finds them via `model://`.
