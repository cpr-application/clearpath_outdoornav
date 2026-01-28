# Message containing status of individual Waypoint in NetworkMission

uint8 PENDING = 0
uint8 ACTIVE = 1
uint8 SUCCEEDED = 2
uint8 NAV_FAILURE = 3
uint8 TASK_WARNING = 4
uint8 TASK_FAILURE = 5

uint8 state
clearpath_navigation_msgs/Waypoint waypoint
