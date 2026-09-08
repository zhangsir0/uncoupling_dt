# SpeedCommand — 行为树下发的速度控制指令
# 发布频率: 事件驱动
# 发布者: 行为树节点
# 订阅者: speed_track_node + 底盘控制仲裁 (L3 决策)

uint8 target_gare       # 目标档位: 1=P档, 2=R档, 3=N档, 4=D档
uint8 ctrl_mode         # 运动模式: 0=刹车, 1=开始跟车, 2=微减速, 3=恢复跟随, 4=减速停车, 10=前进定速+到点, 11=后退定速+到点
uint8 target_id         # 目标车厢 ID
uint8  coupling_count   # 连挂数
float32 position        # 相对目标距离
float32 max_forward_vel # 最大前进速度 (m/s)
float32 max_backward_vel# 最大后退速度 (m/s)
float32 max_acc         # 最大前进加速度 (m/s²)
float32 max_dec         # 最大减速度 (m/s²)
