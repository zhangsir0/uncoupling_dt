cmake_minimum_required(VERSION 3.16)
project(uncoupling_robot_bt)

# C++17 required by BehaviorTree.CPP
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ================================================================
# 依赖
# ================================================================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(data_interfaces REQUIRED)

# ================================================================
# 自定义行为树节点库 (robot_nodes)
# 供 bt_executor_node 和 validate_bt_xml 共用
# ================================================================
add_library(robot_nodes SHARED
  src/robot_nodes.cpp
)

target_include_directories(robot_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_target_dependencies(robot_nodes
  rclcpp
  behaviortree_cpp
  std_msgs
  data_interfaces
)

# ================================================================
# 行为树执行器 (ROS2 Node)
# ================================================================
add_executable(bt_executor_node
  src/bt_executor_node.cpp
)

target_link_libraries(bt_executor_node
  robot_nodes
)

ament_target_dependencies(bt_executor_node
  rclcpp
  behaviortree_cpp
  std_msgs
  data_interfaces
)

# ================================================================
# 行为树 XML 验证工具 (独立可执行文件)
# ================================================================
add_executable(validate_bt_xml
  src/validate_bt_xml.cpp
)

target_link_libraries(validate_bt_xml
  robot_nodes
)

ament_target_dependencies(validate_bt_xml
  rclcpp
  behaviortree_cpp
)



# ================================================================
# 安装
# ================================================================

# 头文件
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# 可执行文件
install(TARGETS
  robot_nodes
  bt_executor_node
  validate_bt_xml
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# 配置文件 (XML 行为树定义)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Launch 文件
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Python 脚本 (Mock GUI 测试工具)
install(PROGRAMS scripts/bt_mock_gui.py
  DESTINATION lib/${PROJECT_NAME}
)
# 黑板实时监控工具
install(PROGRAMS scripts/bt_blackboard_watch.py
  DESTINATION lib/${PROJECT_NAME}
)

# ================================================================
# 测试 (可选)
# ================================================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
