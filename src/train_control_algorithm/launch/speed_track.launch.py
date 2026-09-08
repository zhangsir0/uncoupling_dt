#!/usr/bin/env python3
"""启动 speed_track_node (PID 基础版)。

用法:
    ros2 launch train_control_algorithm speed_track.launch.py

日志自动保存到: ~/workspace/log/speed_track_YYYYMMDD_HHMMSS.log
"""

import os
import time
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo


_PKG = "train_control_algorithm"
_PKG_SHARE = get_package_share_directory(_PKG)
_CONFIG = os.path.join(_PKG_SHARE, "config", "speed_track_params.yaml")

_LOG_DIR = os.path.expanduser("~/workspace/log")
Path(_LOG_DIR).mkdir(parents=True, exist_ok=True)
_LOG_FILE = os.path.join(
    _LOG_DIR,
    f"speed_track_{time.strftime('%Y%m%d_%H%M%S')}.log",
)


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=f"日志文件: {_LOG_FILE}"),

        ExecuteProcess(
            cmd=[
                "bash", "-c",
                f"ros2 run {_PKG} speed_track_node "
                f"--ros-args --params-file {_CONFIG} --log-level INFO "
                f"2>&1 | tee {_LOG_FILE}"
            ],
            output="screen",
            name="speed_track_node",
        ),
    ])
