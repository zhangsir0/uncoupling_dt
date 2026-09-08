# BtHmiState — 行为树统一状态 (发布给 HMI 人机界面)
# 发布频率: 10 Hz
# 发布者: bt_executor_node
# 订阅者: HMI 人机交互

uint8 system_mode
# 0 = MANUAL      — HMI 手动控制 (行为树已停止, hmi_ctrl_mode=0)
# 1 = AUTO_DYNAMIC — 自动动态摘钩 (work_mode=1, MPC 追车)
# 2 = AUTO_STATIC  — 自动静态摘钩 (work_mode=0, PID 导航)
# 3 = E_STOP       — 紧急停止 (E_STOP 序列执行中)

string active_subtree
# "IDLE" | "S0_INIT" | "S1_IDLE" | "S2_ARM_CHECK" |
# "S3_NAV_WAIT_AREA" | "S4_TARGET_POSITIONING" |
# "S5_TRACK_APPROACH" | "S6_SYNC_STABILIZE" | "S7_ARM_DEPLOY" |
# "S8_ATTACH_DECEL" | "S9_UNCOUPLE" | "S11_ARM_RESET" |
# "S5_STATIC_ALIGN" | "S9_STATIC_UNCOUPLE" | "S11_STATIC_RESET" |
# "S12_RETURN_HOME" | "E_STOP"

int32 remaining_tasks   # 剩余摘钩任务数 (-1=未知)
int32 detect_id         # 当前间隙计数 (-1=未知)
string active_leaf_node # 当前执行的叶子节点名称 (如 "WaitForCIPSTask")
