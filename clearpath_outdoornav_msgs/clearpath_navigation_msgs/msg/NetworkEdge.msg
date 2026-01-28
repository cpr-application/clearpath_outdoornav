# A directed edge connecting two NetworkPoint instances

float32 MIN_COST_FACTOR = 0.0
float32 MAX_COST_FACTOR = 100.0
float32 DEFAULT_COST_FACTOR = 1.0

# The unique ID for this edge
string uuid

# The points on either end of this edge
clearpath_navigation_msgs/NetworkPoint start_point
clearpath_navigation_msgs/NetworkPoint end_point

# The deviation radius allowed around this edge in meters
# If zero/negative, the planner will use the parent NetworkGraph object's default_radius property
float32 radius

# The speed limit for the zone covered by this edge in m/s
# If zero/negative, the planner will use the parent NetworkGraph object's default_speed_limit property
float32 speed_limit

# An additional cost factor to prioritize certain edges over others in the planner
# Higher costs -> edge is less likely to be included in the planned route
# Expected range is (0.0, 100.0] (i.e. 0 < cost_factor <= 100).
# Values outside the expected range will be treated as the default 1.0
float32 cost_factor
