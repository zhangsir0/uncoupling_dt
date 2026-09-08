/**
 * @file validate_bt_xml.cpp
 * @brief 行为树 XML 验证工具
 *
 * ## 功能
 * 独立验证行为树 XML 文件的结构正确性:
 *   1. XML 语法是否正确
 *   2. 引用的节点类型是否已注册
 *   3. 树结构是否合法 (非空, 无循环引用等)
 *   4. 打印完整的树结构到终端
 *
 * ## 用法
 *
 * ### 编译
 *   cd ~/colcon_ws
 *   colcon build --packages-select uncoupling_robot_bt
 *
 * ### 运行 (源码路径)
 *   ./build/uncoupling_robot_bt/validate_bt_xml \
 *     src/uncoupling_robot_bt/config/uncoupling_robot_bt.xml
 *
 * ### 运行 (Docker 容器内)
 *   ros2 run uncoupling_robot_bt validate_bt_xml \
 *     /path/to/uncoupling_robot_bt.xml
 *
 * ### 不指定路径 (默认搜索)
 *   ros2 run uncoupling_robot_bt validate_bt_xml
 *
 * ## 输出示例
 *
 *   验证通过:
 *     [PASS] XML 语法检查通过
 *     [PASS] 节点类型检查通过 (27 个节点类型)
 *     [PASS] 树结构验证通过 (共 62 个节点)
 *     Tree structure:
 *       └─ Sequence [mission_flow]
 *           ├─ SubTree [s0_init]
 *           │   └─ Sequence [power_on_self_check]
 *           │       ├─ CheckCommunication
 *           │       └─ ...
 *           ├─ SubTree [s1_idle]
 *           │   └─ ...
 *
 *   验证失败:
 *     [FAIL] 树结构验证失败: Node "MissingNode" not registered
 */

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "uncoupling_robot_bt/robot_nodes.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

// ================================================================
// 彩色输出辅助
// ================================================================
namespace
{
#define COLOR_GREEN  "\033[0;32m"
#define COLOR_RED    "\033[0;31m"
#define COLOR_YELLOW "\033[1;33m"
#define COLOR_CYAN   "\033[0;36m"
#define COLOR_RESET  "\033[0m"

void pass(const std::string& msg) {
  std::cout << COLOR_GREEN << "  [PASS] " << COLOR_RESET << msg << std::endl;
}

void fail(const std::string& msg) {
  std::cout << COLOR_RED << "  [FAIL] " << COLOR_RESET << msg << std::endl;
}

void warn(const std::string& msg) {
  std::cout << COLOR_YELLOW << "  [WARN] " << COLOR_RESET << msg << std::endl;
}

void info(const std::string& msg) {
  std::cout << COLOR_CYAN << "  [INFO] " << COLOR_RESET << msg << std::endl;
}
}  // namespace

// ================================================================
// 查找 XML 文件 (尝试多个路径)
// ================================================================
std::string findXmlFile(const std::string& user_path)
{
  // 1. 用户指定的路径 (绝对路径或相对于 CWD)
  if (!user_path.empty() && std::filesystem::exists(user_path)) {
    return user_path;
  }

  // 2. 默认搜索路径
  std::vector<std::string> search_paths = {
    // 源码树中的位置
    "/home/sz/colcon_ws/src/uncoupling_robot_bt/config/uncoupling_robot_bt.xml",
    // 相对于 CWD
    "src/uncoupling_robot_bt/config/uncoupling_robot_bt.xml",
    "config/uncoupling_robot_bt.xml",
  };

  for (const auto& p : search_paths) {
    if (std::filesystem::exists(p)) {
      return p;
    }
  }

  return "";
}

// ================================================================
int main(int argc, char** argv)
{
  std::string xml_path;
  if (argc >= 2) {
    xml_path = argv[1];
  }

  std::cout << "\n"
            << "==============================================\n"
            << "  铁路驼峰摘钩机器人 - 行为树 XML 验证工具\n"
            << "==============================================\n\n";

  // ── 步骤 1: 查找 XML 文件 ────────────────────────────────────
  xml_path = findXmlFile(xml_path);
  if (xml_path.empty()) {
    fail("找不到行为树 XML 文件!");
    std::cout << "\n  请指定路径: " << argv[0]
              << " <path_to_xml>\n" << std::endl;
    return 1;
  }
  info("XML 文件: " + xml_path);

  // ── 步骤 2: 创建 Factory 并注册所有节点 ──────────────────────
  BT::BehaviorTreeFactory factory;
  // 不需要 ROS2 node (传 nullptr 可以是因为所有节点逻辑模拟执行)
  // RegisterAllNodes 需要 node 参数，但我们验证时不需要真实 ROS2 通信
  // 这里手动注册来避免依赖 ROS2:
  uncoupling_robot::RegisterAllNodes(factory, nullptr);

  info("已注册 " + std::to_string(factory.manifests().size()) + " 个节点类型");

  // ── 步骤 3: 加载 XML → 检查语法和节点引用 ────────────────────
  bool parse_ok = true;
  try {
    factory.registerBehaviorTreeFromFile(xml_path);
    pass("XML 语法和节点引用检查通过");
  } catch (const std::exception& e) {
    fail("XML 解析失败: " + std::string(e.what()));
    parse_ok = false;
  }

  if (!parse_ok) {
    std::cout << "\n  常见问题:\n"
              << "  1. XML 中包含未注册的节点名称\n"
              << "  2. XML 格式不正确 (缺少 <root> 标签等)\n"
              << "  3. BTCPP_format 版本号错误\n"
              << "  4. 端口类型不匹配\n"
              << "\n  请对照 robot_nodes.h 中的 RegisterAllNodes() 检查节点名。\n"
              << std::endl;
    return 1;
  }

  // ── 步骤 4: 创建树 → 检查结构完整性 ──────────────────────────
  bool tree_ok = true;
  std::unique_ptr<BT::Tree> tree;
  try {
    tree = std::make_unique<BT::Tree>(factory.createTree("MainTree"));
    size_t node_count = 0;
    for (const auto& subtree : tree->subtrees)
      node_count += subtree->nodes.size();
    pass("行为树创建成功, 共 " + std::to_string(node_count) + " 个节点");

    // 检查: 树不能为空
    bool empty = true;
    for (const auto& subtree : tree->subtrees)
      if (!subtree->nodes.empty()) { empty = false; break; }
    if (empty) {
      fail("行为树没有节点!");
      tree_ok = false;
    }

    // 检查: 根节点不能为空
    if (!tree->rootNode()) {
      fail("行为树没有根节点!");
      tree_ok = false;
    }

  } catch (const std::exception& e) {
    fail("创建行为树失败: " + std::string(e.what()));
    tree_ok = false;
  }

  if (!tree_ok) {
    std::cout << "\n  常见问题:\n"
              << "  1. XML 中缺少 ID=\"MainTree\" 的 BehaviorTree\n"
              << "  2. 子树 SubTree ID 引用错误\n"
              << "  3. 黑板条目类型冲突\n"
              << std::endl;
    return 1;
  }

  // ── 步骤 5: 打印树结构 ───────────────────────────────────────
  std::cout << "\n"
            << "----------------------------------------------\n"
            << "  行为树结构\n"
            << "----------------------------------------------\n\n";
  BT::printTreeRecursively(tree->rootNode());

  // ── 步骤 6: 统计信息 ─────────────────────────────────────────
  size_t total = 0;
  for (const auto& subtree : tree->subtrees)
    total += subtree->nodes.size();
  info("总节点数: " + std::to_string(total));

  // ── 步骤 7: 生成 TreeNodesModel (给 Groot2 用) ────────────────
  std::string model_xml = BT::writeTreeNodesModelXML(factory, true);
  std::string model_path = "/tmp/uncoupling_robot_tree_nodes_model.xml";
  std::ofstream(model_path) << model_xml;
  info("TreeNodesModel 已写入: " + model_path +
       " (导入 Groot2 时使用此文件)");

  // ── 完成 ─────────────────────────────────────────────────────
  std::cout << "\n"
            << "==============================================\n"
            << COLOR_GREEN << "  验证通过!" << COLOR_RESET << "\n"
            << "==============================================\n"
            << "\n"
            << "  下一步:\n"
            << "  1. 运行执行器: ros2 run uncoupling_robot_bt bt_executor_node\n"
            << "  2. 用 Groot2 连接: 启动 Groot2 → Monitor → localhost:1667\n"
            << "  3. 导入节点模型: " << model_path << "\n"
            << std::endl;

  return 0;
}
