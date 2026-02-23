cmake_minimum_required(VERSION 3.8)
project(mecanum_task_manager)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY mecanum_task_manager launch config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    mecanum_task_manager/qr_reader.py
    mecanum_task_manager/task_manager.py
    mecanum_task_manager/state_machine.py
    mecanum_task_manager/patrol_manager.py
    mecanum_task_manager/simple_patrol.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

