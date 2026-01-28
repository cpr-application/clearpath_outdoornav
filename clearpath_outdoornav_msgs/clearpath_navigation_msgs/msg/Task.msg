# A single task that can be executed at a Waypoint

# The human-readable name of this task
string name

# A UUID string to uniquely identify this Task
string uuid

# The ROS action that this task executes
string action_server_name

# The version of this task
string version

# Numerical/boolean data to be passed to the action_server_name
# The exact meaning of these values is dependent on the underlying service
float64[] floats

# String data to be passed to the action_server_name
# The exact meaning of these values is dependent on the underlying service
string[] strings

# Boolean value which determines whether a mission using this task can continue
# if this task fails to execute
bool allow_failure
