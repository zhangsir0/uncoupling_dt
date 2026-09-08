#!/usr/bin/env python3
"""
自动提钩通讯脚本 - 底盘工控机 → 机械臂(ARM) / 共速臂(CO-ARM) + cmd_vel

流程:
  INIT                 >> ARM:40001=1,40002=0  >> CO:40001=1,40003=0
  WAIT_BOTH_READY      等 ARM=2 + CO=2 → cmd_vel:0.2x3s → >> CO:40001=3,40008=1
  CO_RUNNING           监控 CO: =4运动中 =5充磁(cmd_vel:0.2x3s) =6吸附(cmd_vel:0.2x3s→发ARM)
  WAIT_ARM             监控 ARM: =4运行中 =5提钩OK→复位→等ARM=2→消磁→cmd_vel:0.2→收回
  CO_RETRACT           等 CO=2 → cmd_vel:0 → 作业结束
  DONE
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import Twist
import threading


def get_reg_value(status_msg, reg_addr):
    for i in range(0, len(status_msg.data) - 1, 2):
        if int(status_msg.data[i]) == reg_addr:
            return int(status_msg.data[i + 1])
    return None


class AutoUnhookNode(Node):
    def __init__(self):
        super().__init__('auto_unhook_node')

        # --- Publishers ---
        self.arm_cmd_pub = self.create_publisher(Int32MultiArray, '/arm_cmd', 10)
        self.co_arm_cmd_pub = self.create_publisher(Int32MultiArray, '/co_arm_cmd', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Subscribers ---
        self.arm_status_sub = self.create_subscription(
            Int32MultiArray, '/arm_status', self.arm_status_cb, 10)
        self.co_arm_status_sub = self.create_subscription(
            Float32MultiArray, '/co_arm_status', self.co_arm_status_cb, 10)

        # --- State ---
        self.arm_40001 = None
        self.co_arm_40001 = None
        self.lock = threading.Lock()

        self.state = "INIT"
        self.event_seen = set()
        self.cmd_vel_timer = None

        self.get_logger().info("=" * 60)
        self.get_logger().info("  自动提钩脚本启动")
        self.get_logger().info("=" * 60)

        self.create_timer(1.0, self._start)

    # ==================== 状态回调 ====================
    def arm_status_cb(self, msg):
        v1 = get_reg_value(msg, 40001)
        v2 = get_reg_value(msg, 40002)
        v3 = get_reg_value(msg, 40003)
        v20 = get_reg_value(msg, 40020)
        if v1 is not None:
            with self.lock:
                old = self.arm_40001
                self.arm_40001 = v1
            if old != v1:
                self.get_logger().info(f"[ARM 实时] 40001={v1}  40002={v2}  40003={v3}  40020={v20}")

    def co_arm_status_cb(self, msg):
        val = get_reg_value(msg, 40001)
        if val is not None:
            with self.lock:
                old = self.co_arm_40001
                self.co_arm_40001 = val
            if old != val:
                self.get_logger().info(f"[CO  实时] 40001 = {val}")

    # ==================== 命令发送（每次只发一下） ====================
    def _pub_arm(self, reg, val):
        msg = Int32MultiArray()
        msg.data = [reg, val]
        self.arm_cmd_pub.publish(msg)
        self.get_logger().info(f">> ARM  : {reg}={val}")

    def _pub_co(self, reg, val):
        msg = Int32MultiArray()
        msg.data = [reg, val]
        self.co_arm_cmd_pub.publish(msg)
        self.get_logger().info(f">> CO   : {reg}={val}")

    # ==================== cmd_vel 50Hz 不间断 ====================
    def _start_cmd_vel_loop(self):
        self._cv_speed = 0.0
        self._cv_remaining = 0.0
        self._cv_on_done = None
        dt = 0.02

        def tick():
            self._cv_remaining = max(0.0, self._cv_remaining - dt)
            if self._cv_remaining <= 0 and self._cv_on_done is not None:
                cb = self._cv_on_done
                self._cv_on_done = None
                cb()

            t = Twist()
            t.linear.x = self._cv_speed
            self.cmd_vel_pub.publish(t)

        self.cmd_vel_timer = self.create_timer(dt, tick)

    def _set_cv(self, speed, duration, on_done=None):
        self.get_logger().info(f"[cmd_vel] x={speed} m/s, {duration}s")
        self._cv_speed = speed
        self._cv_remaining = duration
        self._cv_on_done = on_done

    # ==================== 一次性延时 ====================
    def _once(self, seconds, callback):
        def _oneshot():
            callback()
            timer.cancel()
        timer = self.create_timer(seconds, _oneshot)

    # ==================== 状态机 ====================
    def _start(self):
        self._tick = self.create_timer(0.2, self._run)

    def _run(self):
        with self.lock:
            arm = self.arm_40001
            co = self.co_arm_40001

        s = self.state

        if s == "INIT":
            self._st_init()
        elif s == "WAIT_BOTH_READY":
            self._st_wait_ready(arm, co)
        elif s == "CO_RUNNING":
            self._st_co_running(co)
        elif s == "WAIT_ARM":
            self._st_wait_arm(arm)
        elif s == "CO_RETRACT":
            self._st_co_retract(co)
        elif s in ("DELAY", "CMD_VEL_BUSY", "DONE"):
            pass

    # ==================== INIT ====================
    def _st_init(self):
        self._pub_arm(40001, 1)
        self._pub_arm(40002, 0)
        self._pub_co(40001, 1)
        self._pub_co(40003, 0)
        self._start_cmd_vel_loop()
        self.get_logger().info("初始命令已发送，等待 ARM=2 且 CO=2 ...")
        self.state = "WAIT_BOTH_READY"

    # ==================== WAIT_BOTH_READY ====================
    def _st_wait_ready(self, arm, co):
        if arm == 2 and "arm_ready" not in self.event_seen:
            self.event_seen.add("arm_ready")
            self.get_logger().info("[ARM] 机械臂回位完成! (40001=2)")

        if co == 2 and "co_ready" not in self.event_seen:
            self.event_seen.add("co_ready")
            self.get_logger().info("[CO ] 共速臂回位完成! (40001=2)")

        if arm == 2 and co == 2 and "both_ready" not in self.event_seen:
            self.event_seen.add("both_ready")
            self.get_logger().info(">>> 双回位完成，cmd_vel: x=0.2 持续3秒后搭架...")
            self.state = "CMD_VEL_BUSY"
            self._set_cv(0.2, 3.0, self._on_ready_cv_done)

    def _on_ready_cv_done(self):
        self._pub_co(40001, 3)
        self._pub_co(40008, 1)
        self.get_logger().info("已发送 CO:40001=3 40008=1，监控 CO-ARM 状态...")
        self.state = "CO_RUNNING"

    # ==================== CO_RUNNING ====================
    def _st_co_running(self, co):
        if co is None:
            return

        if 110 <= co <= 199 and f"co_fault_{co}" not in self.event_seen:
            self.event_seen.add(f"co_fault_{co}")
            self.get_logger().error(f"[CO ] 故障码! 40001={co}")

        if co == 4 and "co_run" not in self.event_seen:
            self.event_seen.add("co_run")
            self.get_logger().info("[CO ] 共速臂co-arm运动中 (40001=4)")

        # 充磁 → cmd_vel 0.2 维持3秒
        if co == 5 and "co_5" not in self.event_seen:
            self.event_seen.add("co_5")
            self.get_logger().info("[CO ] 已充磁，底盘开始降速 (40001=5)")
            self.get_logger().info("     cmd_vel: x=0.2 持续3秒...")
            self.state = "CMD_VEL_BUSY"
            self._set_cv(0.2, 3.0, self._on_co5_cv_done)

        # 吸附 → cmd_vel 0.2 维持3秒 → 发ARM提钩
        if co == 6 and "co_6" not in self.event_seen:
            self.event_seen.add("co_6")
            self.get_logger().info("[CO ] 已吸附，底盘挂N档 (40001=6)")
            self.get_logger().info("     cmd_vel: x=0.2 持续3秒后发ARM提钩...")
            self.state = "CMD_VEL_BUSY"
            self._set_cv(0.2, 3.0, self._on_co6_cv_done)

    def _on_co5_cv_done(self):
        self.get_logger().info("降速完成 → CO_RUNNING")
        self.state = "CO_RUNNING"

    def _on_co6_cv_done(self):
        self.get_logger().info("发送 ARM 提钩指令")
        self._pub_arm(40001, 3)
        self._pub_arm(40002, 0)
        self.get_logger().info(">> ARM  : 40001=3 40002=0，等待 ARM 状态变化...")
        self.state = "WAIT_ARM"

    # ==================== WAIT_ARM ====================
    def _st_wait_arm(self, arm):
        if arm is None:
            return

        if arm >= 110 and f"arm_fault_{arm}" not in self.event_seen:
            self.event_seen.add(f"arm_fault_{arm}")
            self.get_logger().error(f"[ARM] 故障码! 40001={arm}")

        if arm == 4 and "arm_run" not in self.event_seen:
            self.event_seen.add("arm_run")
            self.get_logger().info("[ARM] 运行中 (40001=4)")

        if arm == 5 and "arm_ok" not in self.event_seen:
            self.event_seen.add("arm_ok")
            self.get_logger().info("=" * 60)
            self.get_logger().info("  [ARM] 提钩OK，复位完成! (40001=5)")
            self.get_logger().info("=" * 60)
            # 直接消磁 + cmd_vel 0.2 + 收回
            self._pub_co(40001, 7)
            self.get_logger().info(">> CO   : 40001=7 (取消磁吸)")
            self.get_logger().info("     cmd_vel: x=0.2")
            self._set_cv(0.3, 999.0)
            self._pub_co(40001, 1)
            self.get_logger().info(">> CO   : 40001=1 (共速臂开始收回)")
            self.state = "CO_RETRACT"

    # ==================== CO_RETRACT ====================
    def _st_co_retract(self, co):
        if co == 2 and "co_back" not in self.event_seen:
            self.event_seen.add("co_back")
            self.get_logger().info("=" * 60)
            self.get_logger().info("  [CO ] 共速臂收回，作业结束! (40001=2)")
            self.get_logger().info("=" * 60)
            self.get_logger().info("cmd_vel: x=0 停车")
            self._set_cv(0.0, 0.5, self._on_stop_done)

    def _on_stop_done(self):
        if self.cmd_vel_timer is not None:
            self.cmd_vel_timer.cancel()
            self.cmd_vel_timer = None
        self.get_logger().info("流程全部结束")
        self.state = "DONE"


def main():
    rclpy.init()
    node = AutoUnhookNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
