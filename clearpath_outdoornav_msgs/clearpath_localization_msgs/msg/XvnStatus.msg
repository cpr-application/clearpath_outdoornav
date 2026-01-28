std_msgs/Header header
float64 latitude
float64 longitude
float32 yaw
float32 stddev_x
float32 stddev_y
float32 stddev_yaw

uint8 UNKNOWN = 0
uint8 VISION = 1
uint8 VISUAL_INERTIAL = 2
uint8 INERTIAL_GNSS = 3
uint8 VI_GNSS = 4
uint8 fusion_status

uint8 IMU_NOT_CONVERGED = 0
uint8 IMU_CONVERGED = 1
uint8 imu_status

int8 WS_NOT_USED = -1
int8 WS_NOT_CONVERGED = 0
int8 WS_CONVERGED = 1
int8 wheelspeed_status

uint8 GNSS_UNKNOWN = 0
uint8 GNSS_NO_FIX = 1
uint8 GNSS_DEAD_RECKONING = 2
uint8 GNSS_TIME_FIX = 3
uint8 GNSS_2D_FIX = 4
uint8 GNSS_3D_FIX = 5
uint8 GNSS_3D_WITH_DR = 6
uint8 GNSS_RTK_FLOAT = 7
uint8 GNSS_RTK_FIX = 8

uint8 gnss1_status
uint8 gnss2_status

uint8 RTK_ERROR = 0
uint8 RTK_WARNING = 1
uint8 RTK_CONNECTING = 2
uint8 RTK_CONNECTED = 3
uint8 rtk_status
