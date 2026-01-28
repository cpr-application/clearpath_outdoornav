# Message definition containing the information of a single dock. This includes identifiers, lat/lon and orientation of the dock.

# Message definition containing the information of a single dock. This includes identifiers, lat/lon and orientation of the dock.

string name
string target_name
geometry_msgs/Pose2D t_target_dock
geometry_msgs/Pose2D t_dock_predock
geometry_msgs/Pose2D t_dock_undock
# a300_side_dock:
#     target_name: 'side_dock_target'
#     T_target_dock: {x: -0.5, y: 0.0, yaw: 0.4}
#     T_dock_predock: {x: -1.0, y: 0.3, yaw: 0.8}