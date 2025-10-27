cmake_minimum_required(VERSION 3.8)
project(op_space_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED CONFIG)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
#find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

add_definitions(-DROS_BUILD_SHARED_LIBS=1)

# --- 2. COSTRUISCI LA LIBRERIA DEL PLUGIN ---
add_library(op_space_controller SHARED
  src/op_space_controller.cpp
)
# Imposta lo standard C++ per la tua libreria
target_compile_features(op_space_controller PUBLIC cxx_std_17)

# Specifica le directory di include per la tua libreria
target_include_directories(op_space_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# --- 3. LINKING DELLE DIPENDENZE (PARTE CRUCIALE) ---
# Usa ament_target_dependencies per le dipendenze ROS "pure"
ament_target_dependencies(op_space_controller PUBLIC
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  pinocchio
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  nav_msgs
  octomap_msgs
  trajectory_msgs
  tf2_eigen
 # tf2_geometry_msgs
)

target_link_libraries(op_space_controller PUBLIC
  Eigen3::Eigen
  tf2_ros::tf2_ros
 # tf2_eigen::tf2_eigen
 # tf2_geometry_msgs::tf2_geometry_msgs
)

# Per visibility_control.h
target_compile_definitions(op_space_controller PRIVATE "OP_SPACE_CONTROLLER_BUILDING_DLL")

# Export as a plugin
pluginlib_export_plugin_description_file(controller_interface op_space_controller_plugin.xml)

# --- Installazione ---
# Installa la libreria del controller compilata
install(TARGETS
    op_space_controller
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin # Per .so su Linux, 'lib' è più appropriato di 'bin'
)

# Installa gli header della tua libreria (perché altri pacchetti possano includerli)
install(
  DIRECTORY include/op_space_controller/
  DESTINATION include/op_space_controller
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Esporta le dipendenze per pacchetti a valle
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  controller_interface
  hardware_interface
  pluginlib
  pinocchio
  geometry_msgs
  sensor_msgs
  tf2_ros
  #tf2_geometry_msgs
  tf2_eigen
  nav_msgs
  octomap_msgs
  trajectory_msgs
)
# Esporta il target della libreria
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

ament_package()
