# Message containing status of individual Waypoint in NetworkMission

std_msgs/Header header
builtin_interfaces/Time last_start
string mission_uuid
string map_uuid
bool paused

uint8 IDLE = 0
uint8 MISSION_DIRECT = 1
uint8 MISSION_BY_ID = 2
uint8 NETWORK_MISSION = 3
uint8 NETWORK_MISSION_FROM_GOAL = 4
uint8 NETWORK_GOTO = 5
uint8 LOCAL_DOCKING = 6
uint8 LOCAL_UNDOCKING = 7
uint8 RADIUS_DOCKING = 8
uint8 NETWORK_DOCKING = 9

uint8 state
