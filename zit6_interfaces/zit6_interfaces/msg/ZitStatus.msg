# ZIT6 AUV 核心状态汇总 (v5.0)
# 对应话题: /zit6/state/status

# 控制层级常量 (control_level)
uint8 LEVEL_NONE = 0      # 无人控/安全停
uint8 LEVEL_POS = 1       # 位置环
uint8 LEVEL_VEL = 2       # 速度环
uint8 LEVEL_FORCE = 3     # 推力环 (直接控制)

# 错误标志位常量 (error_flags)
uint32 ERROR_FORCE_STOP = 1   # Bit0: 强制停机（致命）
uint32 ERROR_SENSOR_FAIL = 2  # Bit1: 传感器故障（IMU/DVL）
uint32 ERROR_VOLTAGE_LOW = 4  # Bit2: 电压异常
uint32 ERROR_COMM_TIMEOUT = 8 # Bit3: 通讯超时

bool is_armed              # 是否已解锁（允许执行推力输出）
uint8 arm_mode             # 解锁模式 (0:默认, 3:遥控模式/绕过导航检查)
uint8 control_level        # 当前控制层级 (见 LEVEL_* 常量)
uint8 ins_state            # 惯导内部状态 (0:待机, 1:粗对准, 2:精对准, 3:SINS/GPS/DVL, 4:SINS/DVL, 5:MRU)
bool navigation_ready      # 导航准备就绪 (惯导、DVL等传感器健康并对准)
float32[6] forces          # 6-DOF 目标推力/力矩 [Fx, Fy, Fz, Mroll, Mpitch, Myaw] (归一化或单位N/Nm)
float32 cycle_time_ms      # 控制主循环真实周期 (单位: ms)
uint32 control_overrun_count # 控制周期超预算次数
uint32 thrust_tx_fail_count  # 推力下发失败次数
float32 battery_voltage    # 电池电压 (单位: V)
uint32 error_flags         # 位域错误/警告标识位 (见 ERROR_* 常量)
