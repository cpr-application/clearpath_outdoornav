# A Network of Paths mission
# Broadly the same as a standard Mission, but with additional fields to allow NoP planning

std_msgs/Header header

string name
string uuid

# The ordered list of waypoints to visit during this mission
clearpath_navigation_msgs/Waypoint[] waypoints

# An array of Tasks to execute on Mission start
clearpath_navigation_msgs/Task[] on_start

# An array of Tasks to execute on Mission stop
# These Tasks will execute regardless of mission success or failure
clearpath_navigation_msgs/Task[] on_stop

# Boolean value indicating whether robot should attempt 'return to dock' behavior
# when battery level drops below return_to_dock_battery_level
bool return_to_dock_enabled

# Dock UUID that the robot should return to for 'return to dock' behavior
string return_to_dock_id

# Value from 0 to 1 indicating what charge level the robot will return to dock at
float32 return_to_dock_battery_level

# Value from 0 to 1 indicating what charge level the robot should resume this mission at
# after returning to dock for recharging
float32 resume_mission_battery_level

# Boolean value indicating whether 'network' level replanning should be performed
# in the event that a network edge is deemed un-traversable (e.g., due to obstacles)
bool network_replan_enabled

# Configuration parameters for the mission
# Additional details TBD
string onav_config
