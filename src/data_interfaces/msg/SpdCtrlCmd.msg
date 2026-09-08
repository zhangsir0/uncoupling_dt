# SpdCtrlCmd — 速度控制仲裁后的统一速度指令 (L3 融合输出)
# 发布频率: 50 Hz
# 发布者: 底盘控制仲裁
# 订阅者: BtChassisForwarder 或 yhs_can_control

uint8 ctrl_gare     # 目标档位: 1=P档, 2=R档, 3=N档, 4=D档 (保留, 实际不使用)
float32 ctrl_speed  # 前进/后退速度 (m/s, 正=前进, 负=后退)
uint8 ctrl_break    # 刹车踏板输出 (0-100)
