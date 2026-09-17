#!/usr/bin/env python3
"""
多通道速度/距离可视化节点 (Agg 后端 — 无需 GUI / X11 / tornado)

实时绘制曲线，每次更新保存 PNG。浏览器可直接打开查看。

左轴 (velocity m/s):
  - /ctrl_speed        → CtrlSpeed.ctrl_speed
  - /auto_spd_ctrl_cmd  → AutoSpdCtrlCmd.ctrl_cmd_velocity
  - /chassis_info_fb    → ChassisInfoFb.ctrl_fb.ctrl_fb_velocity
  - /detect_result      → DetectResult.relative_velocity

右轴 (distance m):
  - /detect_result      → DetectResult.current_distance
  - /speed_command      → SpeedCommand.position

用法:
  ros2 run train_control_algorithm velocity_visualizer
  ros2 run train_control_algorithm velocity_visualizer --ros-args -p window_seconds:=30.0 -p output_path:=/tmp/velocity.png
"""

import math
import os
import threading
import time
from collections import deque
from dataclasses import dataclass
from http.server import HTTPServer, BaseHTTPRequestHandler

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from data_interfaces.msg import CtrlSpeed, DetectResult, SpeedCommand
from yhs_can_interfaces.msg import AutoSpdCtrlCmd, ChassisInfoFb


# ═════════════════════════════════════════════════════════════════
#  HTML 页面模板 — 自动刷新图表
# ═════════════════════════════════════════════════════════════════
_HTML_TEMPLATE = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Velocity & Distance Monitor</title>
<style>
  * { margin: 0; padding: 0; box-sizing: border-box; }
  body { background: #0d1117; color: #c9d1d9; font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', sans-serif;
         display: flex; flex-direction: column; align-items: center; padding: 16px; min-height: 100vh; }
  h1 { font-size: 1.2rem; font-weight: 600; margin-bottom: 8px; color: #e6edf3; }
  #container { width: 100%; max-width: 1200px; background: #161b22; border: 1px solid #30363d;
               border-radius: 8px; padding: 12px; }
  #chart { width: 100%; height: auto; display: block; }
  #status { display: flex; justify-content: space-between; align-items: center; margin-top: 8px;
            font-size: 0.8rem; color: #8b949e; }
  .dot { display: inline-block; width: 8px; height: 8px; border-radius: 50%; margin-right: 4px; }
  .dot.live { background: #3fb950; }
  .dot.stale { background: #d29922; }
</style>
</head>
<body>
  <h1>底盘速度 & 距离实时监控</h1>
  <div id="container">
    <img id="chart" src="/chart.png" alt="velocity chart">
    <div id="status">
      <span><span id="led" class="dot live"></span><span id="time_label">--</span></span>
      <span>刷新间隔 0.5s · 窗口 __WINDOW_S__s</span>
    </div>
  </div>
  <script>
    const img = document.getElementById('chart');
    const led = document.getElementById('led');
    const tl  = document.getElementById('time_label');
    let failCount = 0;
    function refresh() {
      const u = '/chart.png?_=' + Date.now();
      img.src = u;
      tl.textContent = new Date().toLocaleTimeString('zh-CN');
      led.className = 'dot live';
    }
    img.onerror = function() {
      failCount++;
      led.className = 'dot stale';
      if (failCount > 5) tl.textContent = '等待数据...';
    };
    img.onload = function() { failCount = 0; };
    setInterval(refresh, 500);
    refresh();
  </script>
</body>
</html>"""


@dataclass
class SeriesConfig:
    """单条曲线的配置"""
    key: str
    label: str
    axis: str          # "vel" = 左轴 (m/s), "dist" = 右轴 (m)
    color: str
    linestyle: str = "solid"


# ── 曲线定义 ──────────────────────────────────────────────
SERIES: list[SeriesConfig] = [
    # 左轴 — 速度 (m/s)
    SeriesConfig("ctrl_speed",            "/ctrl_speed",          "vel",  "#00b894", "solid"),
    SeriesConfig("auto_spd_ctrl_cmd",     "/auto_spd_ctrl_cmd",   "vel",  "#0984e3", "solid"),
    SeriesConfig("chassis_info_fb",       "/chassis_info_fb",     "vel",  "#d63031", "solid"),
    SeriesConfig("detect_rel_vel",        "/detect rel_vel",      "vel",  "#e17055", "solid"),
    # 右轴 — 距离 (m)
    SeriesConfig("detect_cur_dist",       "/detect distance",     "dist", "#6c5ce7", "dashed"),
    SeriesConfig("speed_cmd_position",    "/speed_cmd position",  "dist", "#fdcb6e", "dashed"),
]

VEL_KEYS = [s.key for s in SERIES if s.axis == "vel"]
DIST_KEYS = [s.key for s in SERIES if s.axis == "dist"]
ALL_KEYS = [s.key for s in SERIES]

SERIES_MAP = {s.key: s for s in SERIES}


class VelocityVisualizer(Node):
    """订阅六个数据通道，双轴实时绘制速度+距离曲线。"""

    def __init__(self):
        super().__init__("velocity_visualizer")

        # ── 参数 ──────────────────────────────────────────────
        self.declare_parameter("window_seconds", 90.0)
        self.declare_parameter("history_capacity", 50000)
        self.declare_parameter("output_path", "/home/sz/workspace/velocity_monitor.png")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("http_port", 8765)

        self.window_s = self.get_parameter("window_seconds").value
        self.maxlen = self.get_parameter("history_capacity").value
        self.output_path = self.get_parameter("output_path").value
        self.rate_hz = self.get_parameter("publish_rate").value
        self.http_port = self.get_parameter("http_port").value

        # ── 数据缓冲 ──────────────────────────────────────────
        self.data: dict[str, deque[tuple[float, float]]] = {
            k: deque(maxlen=self.maxlen) for k in ALL_KEYS
        }
        self._last_val: dict[str, float | None] = {k: None for k in ALL_KEYS}

        # ── 订阅 ──────────────────────────────────────────────
        self.create_subscription(CtrlSpeed, "/ctrl_speed", self._on_ctrl_speed, 10)
        self.create_subscription(AutoSpdCtrlCmd, "/auto_spd_ctrl_cmd", self._on_auto_spd, 10)
        self.create_subscription(ChassisInfoFb, "/chassis_info_fb", self._on_chassis_fb, 10)
        self.create_subscription(DetectResult, "/detect_result", self._on_detect_result, 10)
        self.create_subscription(SpeedCommand, "/speed_command", self._on_speed_command, 10)

        # ── 复用 figure + 双轴 ────────────────────────────────
        self.fig, self.ax_vel = plt.subplots(figsize=(14, 6))
        self.ax_dist = self.ax_vel.twinx()

        # ── 定时渲染 ──────────────────────────────────────────
        self._tick = 0
        self.create_timer(1.0 / self.rate_hz, self._render_and_save)

        # ── HTTP 服务器（后台线程）──────────────────────────────
        self._httpd: HTTPServer | None = None
        self._start_http_server()

        self.get_logger().info(
            f"多通道可视化已启动 | 窗口={self.window_s:.0f}s 刷新={self.rate_hz:.0f}Hz\n"
            f"  HTTP 服务: http://localhost:{self.http_port}\n"
            f"  输出文件: {self.output_path}\n"
            f"  左轴(m/s): /ctrl_speed · /auto_spd_ctrl_cmd · /chassis_info_fb · /detect_result(rel_vel)\n"
            f"  右轴(m):   /detect_result(distance) · /speed_command(position)"
        )

    # ── 回调 ──────────────────────────────────────────────────
    def _record(self, key: str, value: float):
        self.data[key].append((time.monotonic(), value))
        self._last_val[key] = value

    def _on_ctrl_speed(self, msg: CtrlSpeed):
        self._record("ctrl_speed", float(msg.ctrl_speed))

    def _on_auto_spd(self, msg: AutoSpdCtrlCmd):
        self._record("auto_spd_ctrl_cmd", float(msg.ctrl_cmd_velocity))

    def _on_chassis_fb(self, msg: ChassisInfoFb):
        self._record("chassis_info_fb", float(msg.ctrl_fb.ctrl_fb_velocity))

    def _on_detect_result(self, msg: DetectResult):
        self._record("detect_rel_vel", float(msg.relative_velocity))
        self._record("detect_cur_dist", float(msg.current_distance))

    def _on_speed_command(self, msg: SpeedCommand):
        self._record("speed_cmd_position", float(msg.position))

    # ── 渲染 & 保存 ──────────────────────────────────────────
    def _render_and_save(self):
        now = time.monotonic()
        cutoff = now - self.window_s
        self._tick += 1

        # 清理过期数据
        for key in ALL_KEYS:
            buf = self.data[key]
            while len(buf) > 1 and buf[0][0] < cutoff:
                buf.popleft()

        self.ax_vel.cla()
        self.ax_dist.cla()

        vel_all: list[float] = []
        dist_all: list[float] = []

        for sc in SERIES:
            is_vel = (sc.axis == "vel")
            ax = self.ax_vel if is_vel else self.ax_dist
            buf = self.data[sc.key]

            if not buf:
                if self._last_val[sc.key] is not None:
                    ax.axhline(y=self._last_val[sc.key], color=sc.color,
                               linewidth=1.0, linestyle="--", alpha=0.4)
                ax.plot([], [], color=sc.color, linestyle=sc.linestyle,
                        linewidth=1.3, label=sc.label)
                continue

            ts, vs = zip(*buf)
            rel_ts = [t - now for t in ts]

            # 持有最后一个值到当前时刻
            if rel_ts[-1] < 0:
                rel_ts = list(rel_ts) + [0.0]
                vs = list(vs) + [vs[-1]]

            ax.plot(rel_ts, vs, color=sc.color, linestyle=sc.linestyle,
                    linewidth=1.3, label=sc.label)
            if is_vel:
                vel_all.extend(vs)
            else:
                dist_all.extend(vs)

        # ── 左轴 (速度) ──────────────────────────────────────
        self.ax_vel.set_xlabel("time (s)")
        self.ax_vel.set_ylabel("velocity (m/s)", color="#2d3436")
        self.ax_vel.tick_params(axis="y", labelcolor="#2d3436")
        self.ax_vel.set_xlim(-self.window_s, 0)
        if vel_all:
            vmin, vmax = min(vel_all), max(vel_all)
            margin = max((vmax - vmin) * 0.15, 0.1)
            self.ax_vel.set_ylim(math.floor((vmin - margin) * 2) / 2,
                                 math.ceil((vmax + margin) * 2) / 2)
        else:
            self.ax_vel.set_ylim(-1.0, 3.0)

        # ── 右轴 (距离) ──────────────────────────────────────
        self.ax_dist.set_ylabel("distance (m)", color="#6c5ce7")
        self.ax_dist.tick_params(axis="y", labelcolor="#6c5ce7")
        if dist_all:
            dmin, dmax = min(dist_all), max(dist_all)
            margin = max((dmax - dmin) * 0.15, 0.1)
            self.ax_dist.set_ylim(math.floor((dmin - margin) * 2) / 2,
                                  math.ceil((dmax + margin) * 2) / 2)
        else:
            self.ax_dist.set_ylim(-0.5, 5.0)

        # ── 标题 & 图例 (两轴合并) ───────────────────────────
        self.ax_vel.set_title(
            f"Velocity & Distance Monitor  |  window={self.window_s:.0f}s  tick={self._tick}"
        )
        lines_v, labels_v = self.ax_vel.get_legend_handles_labels()
        lines_d, labels_d = self.ax_dist.get_legend_handles_labels()
        self.ax_vel.legend(
            lines_v + lines_d, labels_v + labels_d,
            loc="upper left", fontsize=7, ncol=2
        )
        self.ax_vel.grid(True, alpha=0.3)

        self.fig.tight_layout()
        tmp = self.output_path + ".partial.png"
        self.fig.savefig(tmp, dpi=100)
        os.replace(tmp, self.output_path)

    # ══════════════════════════════════════════════════════════
    #  HTTP 服务器
    # ══════════════════════════════════════════════════════════

    def _start_http_server(self):
        """在后台线程启动轻量 HTTP 服务器，浏览器访问即可查看实时图表。"""
        output_path = self.output_path
        window_s = self.window_s

        class _Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path == "/" or self.path == "/index.html":
                    html = _HTML_TEMPLATE.replace("__WINDOW_S__", str(int(window_s)))
                    self._respond(200, "text/html; charset=utf-8", html.encode())
                elif self.path.startswith("/chart.png"):
                    try:
                        with open(output_path, "rb") as f:
                            data = f.read()
                        self._respond(200, "image/png", data)
                    except FileNotFoundError:
                        self._respond(404, "text/plain", b"chart not ready")
                else:
                    self._respond(404, "text/plain", b"not found")

            def _respond(self, code, mime, body):
                self.send_response(code)
                self.send_header("Content-Type", mime)
                self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)

            def log_message(self, fmt, *args):
                pass  # 静默 HTTP 日志

        self._httpd = HTTPServer(("0.0.0.0", self.http_port), _Handler)
        self._http_thread = threading.Thread(
            target=self._httpd.serve_forever, daemon=True)
        self._http_thread.start()

    def destroy_node(self):
        # ── 退出前保存高清 PNG ──
        hd_path = self.output_path.replace(".png", "_final.png")
        try:
            self.fig.savefig(hd_path, dpi=300)
            self.get_logger().info(f"高清截图已保存: {hd_path}")
        except Exception as e:
            self.get_logger().warn(f"保存高清截图失败: {e}")

        if self._httpd is not None:
            self._httpd.shutdown()
        plt.close(self.fig)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VelocityVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
