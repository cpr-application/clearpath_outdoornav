################################################################################
## A log representing a single mission execution
################################################################################

# The log is currently active and being modified
int32 STATE_ACTIVE = 0

# The log is old and closed
int32 STATE_CLOSED = 1

# The master record of the log exists, but its data has been deleted from the disk
int32 STATE_DELETED = 2

# The state of the log on disk
int32 state

# The time the log started recording
clearpath_logger_msgs/EventTime start_time

# The time the log stopped recording
clearpath_logger_msgs/EventTime end_time

# This execution's unique ID
string uuid

# A descriptive name for the log to be displayed in the GUI. Does not need to be unique
string name

# The total percentage of the battery used during the execution
# This measures only negative changes, not positive ones
# e.g. if the robot uses 75% of the battery, returns to dock to recharge to full
# and then finishes the mission using another 75% of the battery, this value
# will read 1.5
float32 battery_percent_used

# The total distance driven by the robot (approximate) during the execution (meters)
# The exact path used to calculate the distance may be simplified and subject to
# sensor noise/error
float32 distance_driven

# A custom JSON field for adding additional custom metrics, meta-data, etc... as needed
string custom_fields_json

# The list of events recorded in this execution
clearpath_logger_msgs/Event[] events
