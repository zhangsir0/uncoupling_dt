cmake_minimum_required(VERSION 3.8)
project(data_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SpdCtrlCmd.msg"
  "msg/SpeedCommand.msg"
  "msg/HmiCommand.msg"
  "msg/ArbitrationState.msg"
  "msg/DetectResult.msg"
  "msg/CtrlSpeed.msg"
  "msg/BtHmiState.msg"
  "msg/HookTask.msg"
  "msg/HookTaskArray.msg"
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
