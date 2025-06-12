cmake_minimum_required(VERSION 3.22)
project(ur5_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)

add_executable(moveit src/moveit.cpp)

ament_target_dependencies(moveit
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  moveit_task_constructor_core
)

install(TARGETS
  moveit
  DESTINATION lib/${PROJECT_NAME}
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
