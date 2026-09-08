# 底盘仲裁节点状态输出
# 频率: 10Hz
std_msgs/Header header
uint8   active_level           # 当前生效仲裁等级: 0=无输入, 1=L1, 2=L2, 3=L3, 4=L4
string  active_source          # 生效来源描述
uint8   ctrl_cmd_gear          # 当前输出档位
float32 ctrl_cmd_velocity      # 当前输出速度 m/s
float32 ctrl_cmd_steering      # 当前输出转向角 度
uint8   ctrl_cmd_brake         # 当前输出刹车 0-100
bool    fence_warning           # 围栏减速预警激活 (fence=1)
bool    fence_alarm             # 围栏停车告警激活 (fence=2)
bool    mag_off_track           # 磁导航脱轨激活
bool    hmi_override_active     # HMI手动接管激活
uint8   l4_source               # L4源: 0=无, 1=speed_track, 2=position_nav
bool[7] input_timeout           # 各输入源超时状态 [speed, pos, speed_cmd, hmi, fence, mag_steer, mag_stop]
