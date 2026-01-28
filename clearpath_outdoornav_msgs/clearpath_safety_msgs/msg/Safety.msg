# Safety monitor message telling us if the safety monitor has been triggered and a message
# indicating what caused the trigger

uint8 SAFETY_ESTOP = 0
uint8 SAFETY_SENSOR_TIMEOUT = 1

bool safety_monitor_triggered
uint8 trigger_type
string message
