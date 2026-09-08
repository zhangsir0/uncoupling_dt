#!/usr/bin/env python3
"""
bt_mock_gui.py — 行为树节点 Mock 注入测试工具 (Qt5 GUI)

功能:
  1. 解析 BT XML 文件，提取所有节点名称、类型和所属子树
  2. 分组展示所有节点，为每个节点提供 SUCCESS / FAILURE / RUNNING / RESET 按钮
  3. 搜索过滤: 按节点名快速定位
  4. 通过 /bt/mock/set 话题注入 mock 状态到 bt_executor_node
  5. 订阅 /bt_status 和 /bt_current_state 实时显示行为树运行状态
  6. 日志窗口显示所有 mock 操作和 BT 状态变化
  7. 快捷场景: 一键预设常用测试模式

用法:
  ros2 run uncoupling_robot_bt bt_mock_gui.py
  python3 bt_mock_gui.py --xml /path/to/uncoupling_robot_bt.xml

依赖:
  pip install PyQt5
  ros2 (rclpy, std_msgs)
"""

import sys
import os
import argparse
import xml.etree.ElementTree as ET
from collections import OrderedDict
from datetime import datetime

# ── Qt imports ───────────────────────────────────────────────────────
try:
    from PyQt5.QtWidgets import (
        QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
        QPushButton, QLabel, QTabWidget, QTextEdit, QScrollArea,
        QFrame, QGridLayout, QSplitter, QGroupBox, QCheckBox,
        QStatusBar, QMessageBox, QSizePolicy, QLineEdit, QToolButton
    )
    from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread
    from PyQt5.QtGui import QFont, QColor, QPalette, QTextCursor
except ImportError:
    print("ERROR: PyQt5 not found. Install with: pip install PyQt5")
    sys.exit(1)

# ── ROS2 imports ──────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty, Bool


# ======================================================================
# BT XML Parser — 提取所有节点信息
# ======================================================================
def parse_bt_xml(xml_path: str) -> OrderedDict:
    """
    解析行为树 XML, 返回:
      OrderedDict[str, list[dict]]  — subtree_name → [{name, type, id, depth}]
    """
    tree = ET.parse(xml_path)
    root = tree.getroot()

    subtrees = OrderedDict()

    # 行为树控制节点 (不直接 mock，但需要识别以正确遍历)
    control_nodes = {
        'Sequence', 'Fallback', 'Parallel', 'ReactiveFallback', 'ReactiveSequence',
        'KeepRunningUntilFailure', 'RetryUntilSuccessful',
        'ForceFailure', 'Inverter', 'AlwaysSuccess',
        'AlwaysFailure', 'SubTree', 'Script', 'Decorator',
    }

    for bt_elem in root.findall('BehaviorTree'):
        bt_id = bt_elem.get('ID', 'Unknown')
        nodes = []

        def extract_nodes(elem, depth=0):
            tag = elem.tag

            if tag == 'Action':
                nodes.append({
                    'name': elem.get('ID', '?'),
                    'type': 'Action',
                    'id': elem.get('ID', '?'),
                    'depth': depth,
                })
            elif tag == 'Condition':
                nodes.append({
                    'name': elem.get('ID', '?'),
                    'type': 'Condition',
                    'id': elem.get('ID', '?'),
                    'depth': depth,
                })
            elif tag == 'SubTree':
                nodes.append({
                    'name': f"SubTree: {elem.get('ID', '?')}",
                    'type': 'SubTree',
                    'id': elem.get('ID', '?'),
                    'depth': depth,
                })
            elif tag == 'Script':
                code = (elem.text or '').strip()
                nodes.append({
                    'name': f'Script: {code[:50]}',
                    'type': 'Script',
                    'id': code[:30],
                    'depth': depth,
                })

            # 递归子元素
            for child in elem:
                extract_nodes(child, depth + 1)

        extract_nodes(bt_elem)
        if nodes:
            subtrees[bt_id] = nodes

    return subtrees


# ======================================================================
# ROS2 Mock Publisher Node
# ======================================================================
class MockRosNode(Node):
    """ROS2 节点: 发布 mock 注入指令，订阅 BT 状态"""

    def __init__(self):
        super().__init__('bt_mock_gui_node')

        self.mock_pub = self.create_publisher(String, '/bt/mock/set', 10)
        self.clear_pub = self.create_publisher(Empty, '/bt/mock/clear', 10)

        # 订阅 BT 状态
        self.bt_status = "IDLE"
        self.bt_current_node = ""
        self.status_sub = self.create_subscription(
            String, '/bt_status', self._on_status, 10)
        self.state_sub = self.create_subscription(
            String, '/bt_current_state', self._on_state, 10)

        # 跟踪当前活跃的 mock
        self.active_mocks = {}  # node_name -> status_str

        self.get_logger().info("BT Mock GUI ROS2 节点已启动")

    def _on_status(self, msg):
        self.bt_status = msg.data

    def _on_state(self, msg):
        self.bt_current_node = msg.data

    def inject_mock(self, node_name: str, status: str):
        """注入 mock 状态: node_name:SUCCESS / node_name:FAILURE 等"""
        msg = String()
        msg.data = f"{node_name}:{status}"
        self.mock_pub.publish(msg)
        if status == 'RESET':
            self.active_mocks.pop(node_name, None)
        else:
            self.active_mocks[node_name] = status

    def clear_all_mocks(self):
        """清除所有 mock 注入"""
        msg = Empty()
        self.clear_pub.publish(msg)
        self.active_mocks.clear()


# ======================================================================
# 单个节点控制器 Widget
# ======================================================================
class NodeControlWidget(QFrame):
    """单个 BT 节点的 mock 控制面板"""

    mock_requested = pyqtSignal(str, str)  # (node_name, status_str)

    STATUS_COLORS = {
        'NONE':    '#555555',
        'SUCCESS': '#2e7d32',
        'FAILURE': '#c62828',
        'RUNNING': '#1565c0',
    }

    def __init__(self, node_info: dict, parent=None):
        super().__init__(parent)
        self.node_name = node_info['name']
        self.node_type = node_info['type']
        self.current_mock = 'NONE'

        self.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        self.setLineWidth(1)

        # 根据深度设置左边距缩进
        depth = node_info.get('depth', 0)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(6 + depth * 8, 2, 6, 2)
        layout.setSpacing(4)

        # 节点名称标签
        name_label = QLabel(self.node_name)
        name_label.setMinimumWidth(180)
        name_label.setFont(QFont("Monospace", 9))
        if self.node_type == 'SubTree':
            name_label.setStyleSheet("color: #ce93d8; font-weight: bold;")
        elif self.node_type == 'Script':
            name_label.setStyleSheet("color: #ffab40; font-size: 9px;")
        elif self.node_type == 'Condition':
            name_label.setStyleSheet("color: #81d4fa;")
        elif self.node_type == 'Action':
            name_label.setStyleSheet("color: #a5d6a7;")
        layout.addWidget(name_label)

        # 类型标签
        type_short = {'Action': 'Act', 'Condition': 'Cnd', 'SubTree': 'Sub', 'Script': 'Scr'}
        type_colors = {'Action': '#388e3c', 'Condition': '#1976d2', 'SubTree': '#7b1fa2', 'Script': '#e65100'}
        type_label = QLabel(type_short.get(self.node_type, '?'))
        type_label.setFixedWidth(28)
        type_label.setFont(QFont("Monospace", 7, QFont.Bold))
        type_label.setAlignment(Qt.AlignCenter)
        type_label.setStyleSheet(
            f"background: {type_colors.get(self.node_type, '#424242')};"
            f"color: white; border-radius: 2px; padding: 1px;")
        layout.addWidget(type_label)

        layout.addStretch()

        # Mock 状态指示器
        self.status_indicator = QLabel("●")
        self.status_indicator.setFont(QFont("Monospace", 12))
        self.status_indicator.setFixedWidth(20)
        self.status_indicator.setAlignment(Qt.AlignCenter)
        self.status_indicator.setStyleSheet(
            f"color: {self.STATUS_COLORS['NONE']};")
        self.status_indicator.setToolTip("无 mock")
        layout.addWidget(self.status_indicator)

        # ── 按钮组 ──
        btn_style = "padding: 2px 6px; font-size: 9px; font-weight: bold; border-radius: 3px;"

        btn_success = QPushButton("✓")
        btn_success.setFixedWidth(28)
        btn_success.setToolTip("注入 SUCCESS")
        btn_success.setStyleSheet(btn_style + "background: #2e7d32; color: white;")
        btn_success.clicked.connect(lambda: self._inject('SUCCESS'))
        layout.addWidget(btn_success)

        btn_failure = QPushButton("✗")
        btn_failure.setFixedWidth(28)
        btn_failure.setToolTip("注入 FAILURE")
        btn_failure.setStyleSheet(btn_style + "background: #c62828; color: white;")
        btn_failure.clicked.connect(lambda: self._inject('FAILURE'))
        layout.addWidget(btn_failure)

        btn_running = QPushButton("⟳")
        btn_running.setFixedWidth(28)
        btn_running.setToolTip("注入 RUNNING")
        btn_running.setStyleSheet(btn_style + "background: #1565c0; color: white;")
        btn_running.clicked.connect(lambda: self._inject('RUNNING'))
        layout.addWidget(btn_running)

        btn_reset = QPushButton("↺")
        btn_reset.setFixedWidth(28)
        btn_reset.setToolTip("清除 mock (恢复实际逻辑)")
        btn_reset.setStyleSheet(btn_style + "background: #616161; color: white;")
        btn_reset.clicked.connect(lambda: self._inject('RESET'))
        layout.addWidget(btn_reset)

    def _inject(self, status: str):
        if status == 'RESET':
            self.current_mock = 'NONE'
        else:
            self.current_mock = status
        self._update_indicator()
        self.mock_requested.emit(self.node_name, status)

    def _update_indicator(self):
        color = self.STATUS_COLORS.get(self.current_mock, '#555555')
        tooltip = {
            'NONE': '无 mock (使用实际逻辑)',
            'SUCCESS': '已注入 SUCCESS',
            'FAILURE': '已注入 FAILURE',
            'RUNNING': '已注入 RUNNING',
        }.get(self.current_mock, '')
        self.status_indicator.setStyleSheet(f"color: {color};")
        self.status_indicator.setToolTip(tooltip)

    def set_visible_by_filter(self, text: str):
        """根据搜索文本显示/隐藏"""
        if not text:
            self.setVisible(True)
        else:
            self.setVisible(text.lower() in self.node_name.lower())


# ======================================================================
# 子树分组面板
# ======================================================================
class SubtreePanel(QWidget):
    """一个子树的所有节点控制面板，带搜索"""

    def __init__(self, subtree_name: str, nodes: list, parent=None):
        super().__init__(parent)
        self.subtree_name = subtree_name
        self.node_controls = []  # 保存所有 NodeControlWidget 引用

        layout = QVBoxLayout(self)
        layout.setSpacing(2)
        layout.setContentsMargins(4, 4, 4, 4)

        # 子树标题
        action_count = sum(1 for n in nodes if n['type'] == 'Action')
        cond_count = sum(1 for n in nodes if n['type'] == 'Condition')
        header = QLabel(
            f"📁 {subtree_name}  "
            f"({len(nodes)} 节点: {action_count} Action, {cond_count} Condition)")
        header.setFont(QFont("Monospace", 10, QFont.Bold))
        header.setStyleSheet(
            "color: #ffab00; padding: 4px 8px; background: #263238; border-radius: 4px;")
        layout.addWidget(header)

        # 搜索栏
        search_layout = QHBoxLayout()
        search_icon = QLabel("🔍")
        search_icon.setStyleSheet("background: transparent;")
        search_layout.addWidget(search_icon)
        self.search_input = QLineEdit()
        self.search_input.setPlaceholderText("搜索节点名... (如 IsCoArm, Lock, E_STOP)")
        self.search_input.setStyleSheet(
            "background: #2a2a2a; color: #e0e0e0; border: 1px solid #444;"
            "padding: 4px 8px; border-radius: 3px; font-family: Monospace;")
        self.search_input.textChanged.connect(self._on_search)
        search_layout.addWidget(self.search_input)

        clear_btn = QPushButton("✕")
        clear_btn.setFixedWidth(24)
        clear_btn.setToolTip("清除搜索")
        clear_btn.clicked.connect(lambda: self.search_input.clear())
        search_layout.addWidget(clear_btn)

        layout.addLayout(search_layout)

        # 节点列表 (滚动)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setStyleSheet("QScrollArea { border: none; background: #1a1a1a; }")
        scroll_widget = QWidget()
        self.scroll_layout = QVBoxLayout(scroll_widget)
        self.scroll_layout.setSpacing(1)
        self.scroll_layout.setContentsMargins(0, 0, 0, 0)

        for node in nodes:
            ctrl = NodeControlWidget(node)
            self.scroll_layout.addWidget(ctrl)
            self.node_controls.append(ctrl)

        self.scroll_layout.addStretch()
        scroll.setWidget(scroll_widget)
        layout.addWidget(scroll)

        # 底部统计
        self.filter_label = QLabel("")
        self.filter_label.setStyleSheet("color: #666; font-size: 9px;")
        layout.addWidget(self.filter_label)

    def _on_search(self, text: str):
        visible_count = 0
        for ctrl in self.node_controls:
            ctrl.set_visible_by_filter(text)
            if not ctrl.isHidden():
                visible_count += 1
        if text:
            self.filter_label.setText(f"显示 {visible_count}/{len(self.node_controls)} 个节点")
        else:
            self.filter_label.setText("")

    def get_active_mock_count(self) -> int:
        """返回当前面板中有活跃 mock 的节点数"""
        return sum(1 for c in self.node_controls if c.current_mock != 'NONE')

    def get_all_controls(self) -> list:
        return self.node_controls


# ======================================================================
# 主窗口
# ======================================================================
class MainWindow(QMainWindow):
    """行为树 Mock 注入主窗口"""

    def __init__(self, ros_node: MockRosNode, subtrees: OrderedDict):
        super().__init__()
        self.ros_node = ros_node
        self.subtrees = subtrees
        self.panels = {}   # subtree_name -> SubtreePanel
        self.node_controls = {}  # node_name -> NodeControlWidget
        self._init_ui()

        # ── 定时器 ──
        self._status_timer = QTimer()
        self._status_timer.timeout.connect(self._update_status)
        self._status_timer.start(200)  # 5 Hz

        self._ros_timer = QTimer()
        self._ros_timer.timeout.connect(self._spin_ros)
        self._ros_timer.start(50)  # 20 Hz

    def _init_ui(self):
        self.setWindowTitle("🚂 摘钩机器人行为树 Mock 注入测试工具")
        self.setMinimumSize(1000, 700)
        self.resize(1100, 850)

        # 暗色主题
        self.setStyleSheet("""
            QMainWindow { background: #121212; }
            QWidget { background: #1e1e1e; color: #e0e0e0; font-size: 11px; }
            QLabel { background: transparent; }
            QPushButton { border: 1px solid #424242; padding: 4px 8px; }
            QPushButton:hover { border: 1px solid #ffab00; }
            QTabWidget::pane { border: 1px solid #333; background: #1a1a1a; }
            QTabBar::tab { background: #2a2a2a; color: #999; padding: 5px 12px;
                           border: 1px solid #333; margin-right: 2px; }
            QTabBar::tab:selected { background: #1a1a1a; color: #ffab00;
                                    border-bottom: 2px solid #ffab00; }
            QScrollArea { border: none; }
            QTextEdit { background: #0d0d0d; color: #b0bec5; font-family: Monospace;
                        font-size: 10px; border: 1px solid #333; }
            QGroupBox { border: 1px solid #333; margin-top: 8px; padding-top: 16px;
                        font-weight: bold; color: #ffab00; }
            QLineEdit { background: #2a2a2a; color: #e0e0e0; border: 1px solid #444;
                        padding: 4px 8px; border-radius: 3px; }
        """)

        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(8, 8, 8, 8)
        main_layout.setSpacing(6)

        # ── 顶部状态栏 ──
        status_frame = QFrame()
        status_frame.setStyleSheet(
            "background: #263238; border-radius: 4px; padding: 6px;")
        status_layout = QHBoxLayout(status_frame)
        status_layout.setContentsMargins(8, 4, 8, 4)

        self.status_label = QLabel("🔴 BT 状态: 未连接")
        self.status_label.setFont(QFont("Monospace", 11, QFont.Bold))
        status_layout.addWidget(self.status_label)

        status_layout.addStretch()

        self.current_node_label = QLabel("当前节点: —")
        self.current_node_label.setFont(QFont("Monospace", 10))
        self.current_node_label.setStyleSheet("color: #81d4fa;")
        status_layout.addWidget(self.current_node_label)

        status_layout.addStretch()

        self.active_mock_label = QLabel("注入: 0")
        self.active_mock_label.setFont(QFont("Monospace", 10))
        self.active_mock_label.setStyleSheet("color: #ffab00;")
        status_layout.addWidget(self.active_mock_label)

        status_layout.addSpacing(12)

        btn_clear_all = QPushButton("🗑 清除全部 Mock")
        btn_clear_all.setStyleSheet(
            "background: #bf360c; color: white; padding: 6px 16px; font-weight: bold; border-radius: 3px;")
        btn_clear_all.clicked.connect(self._clear_all)
        status_layout.addWidget(btn_clear_all)

        main_layout.addWidget(status_frame)

        # ── 快捷场景按钮栏 ──
        scenario_frame = QFrame()
        scenario_frame.setStyleSheet("background: #1a237e; border-radius: 4px; padding: 4px;")
        scenario_layout = QHBoxLayout(scenario_frame)
        scenario_layout.setContentsMargins(8, 4, 8, 4)

        scenario_label = QLabel("🎯 快捷场景:")
        scenario_label.setStyleSheet("color: #90caf9; font-weight: bold;")
        scenario_layout.addWidget(scenario_label)

        scenarios = [
            ("E_STOP 触发", "模拟急停触发", [
                ("IsEStopButtonPressed", "SUCCESS"),
            ]),
            ("E_STOP 全部安全", "所有急停条件返回 FAILURE(安全)", [
                ("IsEStopButtonPressed", "FAILURE"),
                ("IsHeartbeatLost", "FAILURE"),
                ("IsCollisionRisk", "FAILURE"),
                ("IsAtPositionLimit", "FAILURE"),
            ]),
            ("S2 双臂 Home", "双臂 Home 位 OK", [
                ("IsCoArmHome", "SUCCESS"),
                ("IsArmHome", "SUCCESS"),
            ]),
            ("LockTarget 成功", "目标锁定成功", [
                ("LockTargetGap", "SUCCESS"),
            ]),
            ("LockTarget 失败", "目标错过(触发故障)", [
                ("LockTargetGap", "FAILURE"),
            ]),
            ("MPC 收敛", "MPC 追车完成", [
                ("MPCTrackApproach", "SUCCESS"),
            ]),
            ("全部复位", "清除所有 mock", []),
        ]

        for name, tip, _ in scenarios:
            btn = QPushButton(name)
            btn.setToolTip(tip)
            btn.setStyleSheet(
                "background: #283593; color: #e3f2fd; padding: 4px 10px;"
                "border-radius: 3px; font-size: 10px; border: 1px solid #3949ab;")
            # Capture by closure
            btn.clicked.connect(lambda checked, n=name: self._run_scenario(n))
            scenario_layout.addWidget(btn)

        scenario_layout.addStretch()
        main_layout.addWidget(scenario_frame)

        # ── 中央分屏: 节点控制 (上) | 日志 (下) ──
        splitter = QSplitter(Qt.Vertical)

        # 上部: Tab 页 (每个子树一个 tab)
        self.tab_widget = QTabWidget()

        for st_name, st_nodes in self.subtrees.items():
            panel = SubtreePanel(st_name, st_nodes)
            self.tab_widget.addTab(panel, st_name)
            self.panels[st_name] = panel
            # 收集所有 NodeControlWidget
            for ctrl in panel.get_all_controls():
                self.node_controls[ctrl.node_name] = ctrl
                ctrl.mock_requested.connect(self._on_mock_requested)

        splitter.addWidget(self.tab_widget)

        # 下部: 日志窗口
        log_group = QGroupBox("📋 操作日志")
        log_layout = QVBoxLayout(log_group)
        self.log_widget = QTextEdit()
        self.log_widget.setReadOnly(True)
        self.log_widget.document().setMaximumBlockCount(2000)
        log_layout.addWidget(self.log_widget)

        log_ctrl = QHBoxLayout()
        self.auto_scroll_cb = QCheckBox("自动滚动")
        self.auto_scroll_cb.setChecked(True)
        log_ctrl.addWidget(self.auto_scroll_cb)
        log_ctrl.addStretch()
        btn_clear_log = QPushButton("清空日志")
        btn_clear_log.clicked.connect(lambda: self.log_widget.clear())
        log_ctrl.addWidget(btn_clear_log)
        log_layout.addLayout(log_ctrl)

        splitter.addWidget(log_group)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)

        main_layout.addWidget(splitter)

        # ── 底部状态栏 ──
        self.status_bar = QStatusBar()
        self.status_bar.showMessage("就绪 | 等待 ROS2 连接...")
        self.setStatusBar(self.status_bar)

        self._log("=" * 60)
        self._log("🚂 行为树 Mock 注入测试工具 已启动")
        self._log(f"📂 XML 子树数: {len(self.subtrees)}")
        total = sum(len(ns) for ns in self.subtrees.values())
        self._log(f"📊 总节点数: {total}")
        self._log("💡 提示: 使用搜索栏过滤节点 | 快捷场景一键注入")
        self._log("=" * 60)

    # ── 场景预设 ─────────────────────────────────────────────────────
    def _run_scenario(self, name: str):
        """执行快捷场景"""
        scenarios = {
            "E_STOP 触发": [
                ("IsEStopButtonPressed", "SUCCESS"),
            ],
            "E_STOP 全部安全": [
                ("IsEStopButtonPressed", "FAILURE"),
                ("IsHeartbeatLost", "FAILURE"),
                ("IsCollisionRisk", "FAILURE"),
                ("IsAtPositionLimit", "FAILURE"),
            ],
            "S2 双臂 Home": [
                ("IsCoArmHome", "SUCCESS"),
                ("IsArmHome", "SUCCESS"),
            ],
            "LockTarget 成功": [
                ("LockTargetGap", "SUCCESS"),
            ],
            "LockTarget 失败": [
                ("LockTargetGap", "FAILURE"),
            ],
            "MPC 收敛": [
                ("MPCTrackApproach", "SUCCESS"),
            ],
            "全部复位": [],
        }

        if name == "全部复位":
            self._clear_all()
            return

        injections = scenarios.get(name, [])
        self._log(f"🎯 执行场景: {name}")
        for node_name, status in injections:
            self.ros_node.inject_mock(node_name, status)
            # 更新控件指示器
            if node_name in self.node_controls:
                ctrl = self.node_controls[node_name]
                ctrl.current_mock = status
                ctrl._update_indicator()
            emoji = {'SUCCESS': '✅', 'FAILURE': '❌', 'RUNNING': '🔵'}
            self._log(f"  {emoji.get(status, '●')} {node_name} → {status}")

        self._update_active_mock_count()
        self.status_bar.showMessage(
            f"场景: {name} | {len(injections)} 个注入 | {datetime.now().strftime('%H:%M:%S')}")

    # ── Mock 回调 ──────────────────────────────────────────────────────
    def _on_mock_requested(self, node_name: str, status: str):
        """处理 mock 注入请求"""
        if status == 'RESET':
            self.ros_node.inject_mock(node_name, 'RESET')
            self._log(f"🔄 [RESET] {node_name} — 恢复实际逻辑")
        else:
            self.ros_node.inject_mock(node_name, status)
            emoji = {'SUCCESS': '✅', 'FAILURE': '❌', 'RUNNING': '🔵'}
            self._log(f"{emoji.get(status, '●')} [MOCK] {node_name} → {status}")

        self._update_active_mock_count()
        self.status_bar.showMessage(
            f"注入: {node_name} → {status} | {datetime.now().strftime('%H:%M:%S')}")

    def _clear_all(self):
        """清除所有 mock"""
        self.ros_node.clear_all_mocks()
        for ctrl in self.node_controls.values():
            ctrl.current_mock = 'NONE'
            ctrl._update_indicator()
        self._update_active_mock_count()
        self._update_tab_labels()
        self._log("🗑 已清除全部 mock 注入")
        self.status_bar.showMessage("已清除全部 mock")

    def _update_active_mock_count(self):
        """更新活跃 mock 计数和 tab 标签"""
        count = len(self.ros_node.active_mocks)
        self.active_mock_label.setText(f"注入: {count}")
        if count > 0:
            self.active_mock_label.setStyleSheet("color: #ff6f00; font-weight: bold;")
        else:
            self.active_mock_label.setStyleSheet("color: #888;")
        self._update_tab_labels()

    def _update_tab_labels(self):
        """更新 tab 标签显示各子树的活跃 mock 数"""
        for i in range(self.tab_widget.count()):
            tab_name = self.tab_widget.tabText(i).split(" [")[0]  # 去掉旧标记
            panel = self.panels.get(tab_name)
            if panel:
                mock_count = panel.get_active_mock_count()
                if mock_count > 0:
                    self.tab_widget.setTabText(i, f"{tab_name} [{mock_count} 🔴]")
                else:
                    self.tab_widget.setTabText(i, tab_name)

    # ── 状态刷新 ───────────────────────────────────────────────────────
    def _update_status(self):
        """定时刷新 BT 运行状态"""
        bt_status = self.ros_node.bt_status
        current_node = self.ros_node.bt_current_node

        status_color = {
            'IDLE': '#9e9e9e',
            'RUNNING': '#4caf50',
            'SUCCESS': '#2e7d32',
            'FAILURE': '#c62828',
            'E_STOP': '#ff6f00',
        }.get(bt_status, '#9e9e9e')

        self.status_label.setText(
            f"<span style='color:{status_color};'>●</span> "
            f"BT 状态: <b>{bt_status}</b>")
        self.current_node_label.setText(
            f"当前节点: <b>{current_node}</b>" if current_node else "当前节点: —")

        # 同步 mock 计数
        count = len(self.ros_node.active_mocks)
        if int(self.active_mock_label.text().split(": ")[-1]) != count:
            self._update_active_mock_count()

    def _spin_ros(self):
        """执行一次 rclpy spin_once"""
        try:
            rclpy.spin_once(self.ros_node, timeout_sec=0.001)
        except Exception:
            pass

    # ── 日志 ────────────────────────────────────────────────────────────
    def _log(self, text: str):
        """写入日志窗口"""
        ts = datetime.now().strftime('%H:%M:%S')
        self.log_widget.append(f"[{ts}] {text}")
        if self.auto_scroll_cb.isChecked():
            cursor = self.log_widget.textCursor()
            cursor.movePosition(QTextCursor.End)
            self.log_widget.setTextCursor(cursor)

    def closeEvent(self, event):
        """窗口关闭时清理"""
        self._log("👋 关闭 Mock GUI...")
        self._clear_all()
        self._ros_timer.stop()
        self._status_timer.stop()
        self.ros_node.destroy_node()
        event.accept()


# ======================================================================
# main
# ======================================================================
def main():
    parser = argparse.ArgumentParser(description='BT Mock GUI 测试工具')
    parser.add_argument('--xml', type=str, default=None,
                        help='行为树 XML 文件路径')
    args = parser.parse_args()

    # ── 查找 XML 文件 ──
    xml_path = args.xml
    if xml_path is None:
        search_paths = [
            '/home/sz/workspace/src/uncoupling_robot_bt/config/uncoupling_robot_bt.xml',
            '/home/sz/colcon_ws/src/uncoupling_robot_bt/config/uncoupling_robot_bt.xml',
            os.path.join(os.path.dirname(__file__), '..', 'config', 'uncoupling_robot_bt.xml'),
        ]
        for p in search_paths:
            if os.path.exists(p):
                xml_path = p
                break

    if xml_path is None or not os.path.exists(xml_path):
        print("ERROR: 找不到 BT XML 文件!")
        for p in search_paths:
            print(f"  - {p}")
        sys.exit(1)

    print(f"Loading BT XML: {xml_path}")
    subtrees = parse_bt_xml(xml_path)
    total = sum(len(ns) for ns in subtrees.values())
    print(f"Found {len(subtrees)} subtrees, {total} total nodes")
    for name, nodes in subtrees.items():
        actions = sum(1 for n in nodes if n['type'] == 'Action')
        conds = sum(1 for n in nodes if n['type'] == 'Condition')
        print(f"  {name}: {len(nodes)} nodes ({actions} Action, {conds} Condition)")

    # ── 初始化 ROS2 ──
    rclpy.init(args=sys.argv)
    ros_node = MockRosNode()

    # ── Qt App ──
    app = QApplication(sys.argv)
    app.setStyle('Fusion')

    window = MainWindow(ros_node, subtrees)
    window.show()

    ret = app.exec_()

    # ── 清理 ──
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == '__main__':
    main()
