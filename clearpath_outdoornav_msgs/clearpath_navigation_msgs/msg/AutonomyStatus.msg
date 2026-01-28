# Message containing status of individual Waypoint in Mission

std_msgs/Header header
builtin_interfaces/Time last_start
bool paused
string current_goal

uint8 IDLE = 0
uint8 MISSION = 1
uint8 MISSION_FROM_GOAL = 2
uint8 GOTO = 3
uint8 GOTO_POI = 4
uint8 DOCKING_LOCAL = 5
uint8 UNDOCKING_LOCAL = 6
uint8 DOCKING_MAP = 7

uint8 state
