# Message containing status of individual Waypoint in NetworkMission

std_msgs/Header header
builtin_interfaces/Time last_start
bool paused
string current_goal

uint8 IDLE = 0
uint8 NETWORK_MISSION = 1
uint8 NETWORK_MISSION_FROM_GOAL = 2
uint8 NETWORK_GOTO = 3
uint8 NETWORK_GOTO_POI = 4
uint8 LOCAL_DOCKING = 5
uint8 LOCAL_UNDOCKING = 6
uint8 NETWORK_DOCKING = 7

uint8 state
