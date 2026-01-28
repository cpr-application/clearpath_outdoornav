# Message definition containing the information of a single dock. This includes identifiers, lat/lon and orientation of the dock.

# Message definition containing the information of a single dock. This includes identifiers, lat/lon and orientation of the dock.

string name
string dock_template

# location of the dock frame
float64 latitude
float64 longitude
geometry_msgs/Quaternion orientation



# string name
# string uuid
# string frame
# string target_template

# ISO 8601 date/time format of last successfull dock location setting
# string last_location_set_time

# maximum allowable distance to allow for predocking. (ie. the distance within which the dock waypoint must be located)
# float32 max_predock_distance
# the distance from the dock that the robot will navigate to prior to start docking
# float32 predock_distance

# location of the dock
# float64 latitude
# float64 longitude
# float64 heading
