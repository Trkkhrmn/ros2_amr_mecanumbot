cmake_minimum_required(VERSION 3.8)
project(mecanum_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)

# Kinematik kütüphanesi
add_library(mecanum_kinematics SHARED
  src/mecanum_kinematics.cpp
)
target_include_directories(mecanum_kinematics PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Drive node
add_executable(mecanum_drive_node
  src/mecanum_drive_node.cpp
)
target_link_libraries(mecanum_drive_node mecanum_kinematics)
target_include_directories(mecanum_drive_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mecanum_drive_node
  rclcpp geometry_msgs std_msgs sensor_msgs nav_msgs
)

# GTest
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_kinematics
    test/test_kinematics.cpp
  )
  target_link_libraries(test_kinematics mecanum_kinematics)
  target_include_directories(test_kinematics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

install(
  TARGETS mecanum_kinematics mecanum_drive_node
  EXPORT  mecanum_control_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch worlds DESTINATION share/${PROJECT_NAME})
install(
  PROGRAMS
    mecanum_control/lift_controller.py
    mecanum_control/mecanum_kinematics.py
    mecanum_control/wheel_speed_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(mecanum_control_targets HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(mecanum_kinematics)
ament_package()
