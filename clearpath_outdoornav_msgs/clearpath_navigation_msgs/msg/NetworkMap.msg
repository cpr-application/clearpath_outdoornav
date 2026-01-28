# The graph representing the Network of Paths
# This is implemented as a directed graph

string name
string uuid

# The default radius for all edges in the graph
# Edges with a zero/negative radius will use this default instead
float32 default_radius

# The default speed limit for all edges in the graph
# Edges with a zero/negative speed limit will use this default instead
float32 default_speed_limit

# The array of vertices in the graph
clearpath_navigation_msgs/NetworkPoint[] points

# The array of all edges in the graph
clearpath_navigation_msgs/NetworkEdge[] connections
