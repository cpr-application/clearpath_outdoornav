# Single-execution mode; the schedule starts at the designated time and runs once
uint8 MODE_ONCE=0

# Looping mode; the schedule starts at the designated time, and every loop_interval seconds after
# for the designated number of repeats
uint8 MODE_LOOP=1

# Single-shot execution, but does NOT get added to the persistent storage
# Equivalent to MODE_ONCE, but for one-off, throw-away schedules (e.g. "Run X in 5 minutes")
uint8 MODE_TRANSIENT=2

# The human-readable name for this schedule
# Should be unique, but there's no hard requirement for it to be so
string name

# The unique ID of this schedule
# A string of the form "12345678-90ab-cdef-1234-567890abcdef"
string uuid

# The time that this mission starts every day
clearpath_mission_scheduler_msgs/UtcTime start_time

# Either MODE_ONCE or MODE_LOOP to indicate the execution mode
uint8 mode

# In MODE_LOOP, how many times do we repeat the missions?
uint8 loop_repeats

# In MODE_LOOP, how long do we wait between executions?
builtin_interfaces/Duration loop_interval

# The ordered list of Mission UUIDs to execute
string[] mission_ids

# The minimum battery charge needed before we execute the missions
# Must be in the range 0-1
float32 min_battery

# If the missions take longer than this to execute, assume the schedule has failed and alert the user
# Set to zero to disable the timeout
builtin_interfaces/Duration timeout

# Is this schedule enabled?
# Only enabled schedules will execute
bool enabled
