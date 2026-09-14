uint8 axis # 0:X, 1:Y, 2:Z, 3:Roll, 4:Pitch, 5:Yaw
bool is_pos_ring # true: 位置环(含规划器), false: 速度环
float32 kp
float32 ki
float32 kd
float32 i_limit
float32 out_limit

# 规划器参数 (仅在 is_pos_ring 为 true 时有效)
float32 max_v
float32 max_a
