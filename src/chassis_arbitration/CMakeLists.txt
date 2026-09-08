cmake_minimum_required(VERSION 3.8)
project(chassis_arbitration)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yhs_can_interfaces REQUIRED)
find_package(data_interfaces REQUIRED)

include_directories(include)

# ── chassis_arbitration_node ──────────────────────────────────────
add_executable(chassis_arbitration_node
  src/chassis_arbitration_node.cpp
)
ament_target_dependencies(chassis_arbitration_node
  rclcpp
  std_msgs
  yhs_can_interfaces
  data_interfaces
)

install(TARGETS chassis_arbitration_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
