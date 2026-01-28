# A single Waypoint along a Mission

# A UUID string to uniquely identify this Waypoint
string uuid

# The human-readable name of this Waypoint
string name

# The latitude (in degrees) of this Waypoint
float64 latitude

# The longitude (in degrees) of this Waypoint
float64 longitude

# The compass heading (in degrees) of this Waypoint
float64 heading

# A radius in meters indicating the acceptable radius from the target location
# Posititon tolerance is disabled if this value is negative
float64 position_tolerance

# A tolerance in degrees indicating the acceptable deviation from the heading
# Heading tolerance is disabled if this value is negative
float64 yaw_tolerance

# The ordered set of Tasks to execute once the goal position & orientation are reached
clearpath_navigation_msgs/Task[] tasks
