#!/usr/bin/env python3
"""
行为树黑板实时监控工具
用法:
  ros2 run uncoupling_robot_bt bt_blackboard_watch.py          # 打印所有键值对
  ros2 run uncoupling_robot_bt bt_blackboard_watch.py --keys   # 仅显示键名
  ros2 run uncoupling_robot_bt bt_blackboard_watch.py --diff   # 仅显示变化

等价于:
  ros2 topic echo /bt/blackboard | python3 -m json.tool
但更精简, 只显示黑板键值对, 适合终端实时监控
"""

import rclpy
import json
import sys
import argparse
from rclpy.node import Node
from std_msgs.msg import String


class BlackboardWatcher(Node):
    def __init__(self, show_keys_only=False, show_diff=False):
        super().__init__('bt_blackboard_watch')
        self.show_keys_only = show_keys_only
        self.show_diff = show_diff
        self.last_state = {}
        self.sub = self.create_subscription(
            String, '/bt/blackboard', self.cb, 10)
        self.get_logger().info(
            f"监听 /bt/blackboard ... (keys_only={show_keys_only}, diff={show_diff})")

    def cb(self, msg: String):
        data = json.loads(msg.data)
        bbs = data.get('blackboards', {})
        running = data.get('running_nodes', [])

        # 头像
        if not self.show_diff:
            print("\033[2J\033[H", end='')  # 清屏

        # 时间戳
        ts_ns = data.get('timestamp', 0)
        ts_s = ts_ns / 1e9
        print(f"=== 行为树黑板 [{ts_s:.3f}s] ===")

        # 运行中的节点
        if running:
            print(f"\n▶ RUNNING 节点: {running}")

        # 黑板内容
        for bb_name, entries in bbs.items():
            print(f"\n── 黑板: {bb_name} ──")
            if not entries:
                print("  (空)")
                continue

            for key, value in entries.items():
                if self.show_keys_only:
                    print(f"  · {key}")
                    continue

                # 对数值做友好格式化
                if isinstance(value, (int, float)):
                    print(f"  {key:30s} = {value}")
                elif isinstance(value, str):
                    s = value
                    if len(s) > 60:
                        s = s[:57] + '...'
                    print(f"  {key:30s} = \"{s}\"")
                elif isinstance(value, bool):
                    print(f"  {key:30s} = {value}")
                elif isinstance(value, list):
                    if len(value) <= 10:
                        print(f"  {key:30s} = {value}")
                    else:
                        print(f"  {key:30s} = [{len(value)} elements]")
                elif isinstance(value, dict):
                    print(f"  {key:30s} = {{...}}")
                else:
                    print(f"  {key:30s} = {value}")

        # diff 模式
        if self.show_diff:
            changed = {}
            for bb_name, entries in bbs.items():
                for key, value in entries.items():
                    full_key = f"{bb_name}/{key}"
                    if full_key not in self.last_state or self.last_state[full_key] != value:
                        changed[full_key] = value
            if changed:
                print(f"\n△ 变化 ({len(changed)}):")
                for k, v in sorted(changed.items()):
                    old = self.last_state.get(k, '<NEW>')
                    print(f"  {k}: {old} → {v}")
            else:
                print("\n(无变化)")

        # 更新状态
        self.last_state = {}
        for bb_name, entries in bbs.items():
            for key, value in entries.items():
                self.last_state[f"{bb_name}/{key}"] = value


def main():
    parser = argparse.ArgumentParser(description='行为树黑板实时监控')
    parser.add_argument('--keys', action='store_true', help='仅显示键名')
    parser.add_argument('--diff', action='store_true', help='仅显示变化')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = BlackboardWatcher(show_keys_only=args.keys, show_diff=args.diff)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
