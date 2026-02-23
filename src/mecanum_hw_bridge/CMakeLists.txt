cmake_minimum_required(VERSION 3.8)
project(mecanum_hw_bridge)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY mecanum_hw_bridge launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    mecanum_hw_bridge/hw_bridge_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES
    mecanum_hw_bridge/kinematics.py
    mecanum_hw_bridge/__init__.py
  DESTINATION lib/${PROJECT_NAME}
)

# Kinematics unit test: run with
#   PYTHONPATH=$PWD/src/mecanum_hw_bridge python3 src/mecanum_hw_bridge/test/test_kinematics.py
# or: make test (colcon) + CI runs this package's test script explicitly

ament_package()
