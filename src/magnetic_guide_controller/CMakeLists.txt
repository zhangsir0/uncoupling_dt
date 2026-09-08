cmake_minimum_required(VERSION 3.8)
project(magnetic_guide_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(magnetic_guide_driver REQUIRED)
find_package(yhs_can_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(lidar_tf_publisher_node src/lidar_tf_publisher_node.cpp)
add_executable(magnetic_guide_controller_node src/magnetic_guide_controller_node.cpp)
target_include_directories(magnetic_guide_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_include_directories(lidar_tf_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

target_compile_features(magnetic_guide_controller_node PUBLIC c_std_99 cxx_std_17)  # Require C++17

ament_target_dependencies(
  magnetic_guide_controller_node
  "rclcpp"
  "geometry_msgs"
  "ackermann_msgs"
  "magnetic_guide_driver"
  "yhs_can_interfaces"
  "std_msgs"
)
ament_target_dependencies(lidar_tf_publisher_node
  rclcpp
  geometry_msgs
  tf2_ros
)

install(PROGRAMS 
  scripts/teleop_keyboard.py 
  scripts/yhs_velocity_controller.py
  scripts/lidar_fusion_node.py
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS 
  magnetic_guide_controller_node
  lidar_tf_publisher_node
  DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY
  config
  rviz
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()