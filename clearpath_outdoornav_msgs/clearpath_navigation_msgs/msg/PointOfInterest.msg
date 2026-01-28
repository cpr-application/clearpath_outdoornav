# Message containing the data used to represent a Point of interest (POI)

# If this tag is present, the POI can be used as a navigation goal
string TAG_GOTO = goto

# If this tag is present, the POI can be set as the target for a PTZ camera action
string TAG_INSPECT = inspect

# A UUID string to uniquely identify this POI
string uuid

# The human-readable name of this POI
string name

# The latitude (in decimal degrees) of this POI
float64 latitude

# The longitude (in decinal degrees) of this POI
float64 longitude

# The height above ground level (in meters)
float64 elevation

# The compass heading (in degrees) of this POI (optional)
bool enable_heading
float64 heading

# A set of labels assigned to the POI in order to categorize it
string[] tags

# A JSON formatted string that contains user defined POI fields
string custom_fields_json
