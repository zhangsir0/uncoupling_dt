# HmiCommand — HMI 人工遥控指令 (L2 仲裁，覆盖 L4 自动控制)
# 发布频率: 50 Hz
# 发布者: HMI 人机交互
# 订阅者: bt_executor_node (模式控制) + 底盘控制仲裁 (底盘接管)

uint8 hmi_ctrl_mode   # 0=HMI手动控制(行为树停止→S0等待), 1=自动动态摘钩(work_mode=1), 2=自动静态摘钩(work_mode=0), 3=进入ESTOP，4=解除estop
uint8 ctrl_gare       # 目标档位: 1=P档, 2=R档, 3=N档, 4=D档
float32 ctrl_speed    # 前进/后退速度，均为正值 (m/s)
uint8 ctrl_break      # 刹车踏板输出 (0-100)
