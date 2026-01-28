# Message containing current and predicted odom related information

std_msgs/Header header

# Current and predicted odom
nav_msgs/Odometry odom
nav_msgs/Odometry predicted_odom

# Time offset corresponding to prediction
float32 dt
