# USBL有效数据帧

uint32 timestamp_ms          # STM32接收时间戳，单位：ms
uint32 frame_number          # 有效USBL帧计数器

float32 roll_deg             # 基阵横滚角，单位：deg
float32 pitch_deg            # 基阵俯仰角，单位：deg
float32 yaw_deg              # 基阵航向角，单位：deg
float32 pressure_m           # 压力计/深度数据，单位：m

float32[4] slant_range_m     # 四路斜距，单位：m

float32 latitude_deg         # 信标纬度，单位：deg（协议中扩大1e7后转换为int32传输）
float32 longitude_deg        # 信标经度，单位：deg（协议中扩大1e7后转换为int32传输）

float32[3] time_diff         # 通道12、13、14时间差

int16[3] passive_attitude    # USBL被动侧向姿态，单位：0.01deg
                             # [0]: 横滚角 roll
                             # [1]: 俯仰角 pitch
                             # [2]: 航向角 yaw

uint16[4] signal_strength    # 四个接收通道信号强度

uint8[8] energy              # 30~37kHz频段能量，每1kHz一个数据

float32 signal               # 综合信号强度
float32 gain                 # 模拟系统增益

float32 beacon_north_m       # 信标北向位置，单位：m
float32 beacon_east_m        # 信标东向位置，单位：m
float32 beacon_depth_m       # 信标深度，单位：m

uint8 sensor_status          # 传感器状态标志位
                             # bit0: 声通讯更新
                             # bit1: 声通讯有效
                             # bit2: GPS更新
                             # bit3: GPS有效
                             # bit4: SBL/USBL更新
                             # bit5: SBL/USBL有效

uint8 year                   # 年
uint8 month                  # 月
uint8 day                    # 日
uint8 hour                   # 时
uint8 minute                 # 分
float32 second               # 秒

uint8 navigation_status      # 当前导航模式
                             # 0: 待机
                             # 1: 粗对准
                             # 2: 精对准
                             # 3: SINS/GPS/USBL组合导航

uint8 checksum               # USBL帧异或校验
                             # 校验范围：协议0字节~129字节