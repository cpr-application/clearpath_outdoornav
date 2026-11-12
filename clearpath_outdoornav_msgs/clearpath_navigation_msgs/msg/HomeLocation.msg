# The robot's home location, used as the destination of the autonomy/go_home action.
# Home is either a position on the map or one of the docks in the dock database.

# No home location has been set
uint8 TYPE_NONE = 0
# Home is a latitude/longitude position; going home navigates there like a GoTo
uint8 TYPE_LOCATION = 1
# Home is a dock; going home navigates to the dock and docks like a MapDock
uint8 TYPE_DOCK = 2

uint8 type

# TYPE_LOCATION: the position of home (in decimal degrees)
float64 latitude
float64 longitude

# TYPE_LOCATION: the final heading at home (radians, ENU: counter-clockwise from east) (optional)
bool enable_heading
float64 heading

# TYPE_DOCK: the name of the dock in the dock database
string dock_name
