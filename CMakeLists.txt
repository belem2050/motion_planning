cmake_minimum_required(VERSION 3.5)
project(robotic_setup)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(motion_planner src/motion_planner.cpp)
ament_target_dependencies(motion_planner rclcpp geometry_msgs moveit_ros_planning_interface)

add_executable(client src/client.cpp)
ament_target_dependencies(client rclcpp geometry_msgs)

install(TARGETS
  motion_planner
  client
  DESTINATION lib/${PROJECT_NAME})



install(DIRECTORY launch urdf
  DESTINATION share/${PROJECT_NAME})

ament_package()
