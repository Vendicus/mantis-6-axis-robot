cmake_minimum_required(VERSION 3.8)
project(mantis_commander)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)

add_executable(main_planner_node 
  src/main_planner.cpp
  src/trajectory_utils.cpp
)

target_link_libraries(main_planner_node yaml-cpp)

ament_target_dependencies(main_planner_node
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  ament_index_cpp
  tf2
)

add_executable(example_moveit src/example_moveit.cpp)
ament_target_dependencies(example_moveit rclcpp moveit_ros_planning_interface)

install(TARGETS
  example_moveit
  main_planner_node
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
