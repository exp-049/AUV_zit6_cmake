# 全轴 PID 与 规划器状态汇总反馈 (1Hz)

# 位置环 P 参数与规划器限值 [X, Y, Z, Roll, Pitch, Yaw]
float32[6] pos_kp
float32[6] pos_max_v
float32[6] pos_max_a
float32[6] pos_out_limit

# 速度环 PID 参数 [X, Y, Z, Roll, Pitch, Yaw]
float32[6] vel_kp
float32[6] vel_ki
float32[6] vel_kd
float32[6] vel_i_limit
float32[6] vel_out_limit
