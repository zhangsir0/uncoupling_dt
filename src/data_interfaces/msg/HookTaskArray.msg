# HookTaskArray — CIPS 下发的列车摘钩作业指令列表
# 发布频率: 事件驱动
# 发布者: CIPS 平台 → bt_executor_node
# 订阅者: 行为树 (WaitForCIPSTask / ParseTaskMessage)

std_msgs/Header header  # 时间戳和 frame_id
int32 total_hook_num    # 总摘钩数
HookTask[] hook_tasks    # 摘钩任务列表
