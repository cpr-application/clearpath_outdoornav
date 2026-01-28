# Message containing the current navigation state(s). Navigation may be in multiple states.

uint8 STATE_IDLE = 0
uint8 STATE_COMPUTE_PATH = 1
uint8 STATE_EXECUTE_PATH = 2
uint8 STATE_REPLAN = 3
uint8 STATE_NAVIGATING_AROUND_OBSTACLE = 4
uint8 STATE_RECOVERY = 5
uint8 STATE_LOST = 6
uint8 STATE_DONE = 7
uint8 STATE_SAFETY_STOP = 8
uint8 STATE_PAUSE = 9
uint8 STATE_DISABLE = 10

uint8[] states
