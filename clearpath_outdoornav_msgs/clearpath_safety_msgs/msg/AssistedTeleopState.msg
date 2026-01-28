# Assisted teleoperation message indicating the current state of the protection

uint8 STATE_ACTIVE = 0
uint8 STATE_DISABLED = 1
uint8 STATE_COLLISION_IMMINENT = 2
uint8 STATE_STOPPED = 3
uint8 STATE_COLLISION = 4
uint8 STATE_ERROR = 5
uint8 state

uint8 ASSIST_ON = 100
uint8 ASSIST_OFF_SERVICE = 101
uint8 ASSIST_OFF_JOYSTICK_OVERRIDE = 102
uint8 ASSIST_OFF_SENSOR_TIMEOUT = 103
uint8 ASSIST_OFF_REMOTE_CONTROLLER_OVERRIDE = 104
uint8 assist_off_trigger

string error_message
