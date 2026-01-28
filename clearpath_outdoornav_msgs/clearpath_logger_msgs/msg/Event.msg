################################################################################
## Universal log event. Represents a single even recorded to the log
################################################################################

# Tag for events that include a media_path
string TAG_MEDIA = media

# Tag indicating that the event's charger status & battery level have been set
string TAG_POWER = power

# Tag indicating that the event's GPS position has been set
string TAG_LOCATION = location

# Tag indicating that the event's odometry data has been set
string TAG_ODOMETRY = odom

# Tag indicating that the event's velocity data has been set
string TAG_VELOCITY = vel

# Tag indicating that the event's error/debugging data has been set
string TAG_ERROR = error

# Tag indicating that this event was automatically recorded telemetry
string TAG_AUTO_TELEMETRY = auto_telemetry

################################################################################
## Header & general meta-data about the event
################################################################################

# Header
# - seq: the order of this event within its parent ExecutionLog
# - frame_id: the sensor/manipulator frame referenced by the frame_pose field
# - stamp: the time that this event occurred
std_msgs/Header header

################################################################################
## Media & sensor data
################################################################################

# If this even has media, this is its path on-disk
string media_path

# If this event has media, this is its MIME type
# May be left blank
string media_mime_type

# The pose of the sensor/manipulator frame given by header.frame_id relative to
# base_link
geometry_msgs/Pose frame_pose

################################################################################
## Robot location, orientation, velocity
################################################################################

# The robot's latitude, longitude, and heading as recorded by the GPS
# - lat: decimal degrees
# - lon: decimal degrees
# - heading: ENU orientation in radians
geometry_msgs/Point lat_lon_heading

# The robot's location relative to the odometry frame
nav_msgs/Odometry odometry

# The robot's linear & angular velocity
geometry_msgs/Twist velocity

################################################################################
## Power & battery
################################################################################

# The 0-1 charge level of the battery
float32 battery_percent

# True of the robot's battery is charging, otherwise False
bool charging

# True if power-saving/low-power mode is enabled, otherwise False
bool power_saver

################################################################################
## Error and debugging events
################################################################################

# An error code indicating the type of error
int32 error_code

# A diagnostic error message, or non-integer error code
string error_msg

# The name of the module/process/pipeline/etc... that signalled the error
string error_component

################################################################################
## Custom fields for expansion
################################################################################

# Arbitrary free-form tags usable to annotate custom data
string[] tags

# Arbitrary JSON-formatted data that can contain user-defined fields
string custom_fields_json
