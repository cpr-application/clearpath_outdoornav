# Message definition containing the base station survey properties

# ISO 8601 date/time format of last successfull base station survey
string last_survey_time

# Number of samples used in survey
uint32 num_samples

# GPS location of the base station
sensor_msgs/NavSatFix base_location

# denotes a successfull base station survey
bool success
