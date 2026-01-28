# Message definition for an OutdoorNav mission

std_msgs/Header header

# The human-readable name of this mission
string name

# A UUID string used to uniquely identify this mission
string uuid

# The ordered list of Waypoints that make up the mission
clearpath_navigation_msgs/Waypoint[] waypoints

# An array of Tasks to execute on Mission start
clearpath_navigation_msgs/Task[] on_start

# An array of Tasks to execute on Mission stop
# These Tasks will execute regardless of mission success or failure
clearpath_navigation_msgs/Task[] on_stop

# Configuration parameters for the mission
# Additional details TBD
string onav_config
