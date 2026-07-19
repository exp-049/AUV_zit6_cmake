# USBL valid data frame

uint32 timestamp_ms          # STM32 receive timestamp, ms
uint32 frame_number          # valid USBL frame counter

float32 roll_deg             # array attitude roll, deg
float32 pitch_deg            # array attitude pitch, deg
float32 yaw_deg              # array attitude yaw, deg
float32 pressure_m           # array pressure/depth, m
float32[4] slant_range_m     # four slant ranges, m

float32 latitude_deg         # beacon latitude, deg
float32 longitude_deg        # beacon longitude, deg
float32[3] time_diff         # channel time differences
int16[3] passive_attitude    # raw unit: 0.01 deg
uint16[4] signal_strength    # four channel signal strengths
uint8[8] energy              # 30..37 kHz energy

float32 signal               # combined signal strength
float32 gain                 # analog system gain
float32 beacon_north_m       # beacon north position, m
float32 beacon_east_m        # beacon east position, m
float32 beacon_depth_m       # beacon depth, m

uint8 sensor_status          # sensor status bits
uint8 year
uint8 month
uint8 day
uint8 hour
uint8 minute
float32 second
uint8 navigation_status      # current navigation mode
uint8 checksum               # USBL frame checksum
