# Message definition containing the state information of the target tracker.
# This includes whether or not the tracker is active, whether the target is visible/lost, and the target pose and score of that pose.

bool active

# flag indicates if the target is visible or not
bool target_locked

geometry_msgs/PoseWithCovarianceStamped target_pose
float32 score

# the distance of the sensor_link to the dock target
float64 distance_to_target


