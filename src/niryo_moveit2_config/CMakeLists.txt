cmake_minimum_required(VERSION 3.22)
project(niryo_moveit2_config)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(Eigen3 REQUIRED)  #AGGIUNTO

# Aggiungere file C++
add_executable(add_boxes cpp_script/add_boxes.cpp)
add_executable(pilz_trajectory cpp_script/pilz_trajectory.cpp)
add_executable(chomp_traj_a cpp_script/chomp_traj_a.cpp)
add_executable(desired_trajectory_sampling cpp_script/desired_trajectory_sampling.cpp) #AGGIUNTO

# Dipendenze necessarie
ament_target_dependencies(
  add_boxes
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  shape_msgs
  geometry_msgs
  moveit_ros_move_group
)

ament_target_dependencies(
  pilz_trajectory
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  shape_msgs
  geometry_msgs
  moveit_ros_move_group
)

ament_target_dependencies(
  chomp_traj_a
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  shape_msgs
  geometry_msgs
  moveit_ros_move_group
)

# Dipendenze necessarie per 'desired_trajectory_sampling' AGGIUNTO
ament_target_dependencies(
  desired_trajectory_sampling
  rclcpp
  moveit_visual_tools
  moveit_ros_planning_interface
  geometry_msgs
  moveit_ros_move_group
)

# Installa l'eseguibile
install(TARGETS add_boxes pilz_trajectory chomp_traj_a desired_trajectory_sampling
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
