# Motion state message to indicate the current motion, expected motion and the direction of travel of the robot.

# State pertaining to current motion of the robot.
uint8 MOTION_NORMAL=10
uint8 MOTION_STOPPED=11
uint8 MOTION_REVERSE=12
uint8 MOTION_SLOWING=13 # Currently unused
uint8 motion

# State pertaining to the robot's movement along the path. Predictive, ie state
# will be INDICATOR_LEFT_TURN if in or approaching a left turn.
uint8 INDICATOR_NORMAL=20
uint8 INDICATOR_LEFT_TURN=21
uint8 INDICATOR_RIGHT_TURN=22
uint8 INDICATOR_CLOCKWISE=23
uint8 INDICATOR_COUNTERCLOCKWISE=24
uint8 INDICATOR_HAZARD=25 # Currently unused
uint8 indicator

# Direction of travel of the robot relative to its physical forward direction.
uint8 DIRECTION_FORWARD=30
uint8 DIRECTION_REVERSE=31
uint8 direction
