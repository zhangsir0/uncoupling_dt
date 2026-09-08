# HookTask — 单个摘钩任务描述
# 由 CIPS 平台下发, 包含在 HookTaskArray 中

uint8  train_id         # 作业车厢 ID
uint8  train_type_code  # 车型编码
uint8  coupling_count   # 连挂数
bool   air_pipe_off     # 风管脱开状态
bool   carriage_brake   # 抱闸状态
string train_number_str # 车厢号 ASCII 字符串
bool   hook_lever_type  # 提钩杆类型: false=下作用提钩杆, true=上作用提钩杆
