# Coherent MCU odom snapshot. NED position (m, rad), FRD body twist (m/s, rad/s).
float32[6] pose_odom
float32[6] twist_body
# MCU monotonic sample time, wraps after uint32 milliseconds.
uint32 nav_timestamp_ms
bool nav_valid
bool origin_initialized
# Increments on each successful SetOrigin within this MCU boot; zero before init.
uint32 origin_generation
