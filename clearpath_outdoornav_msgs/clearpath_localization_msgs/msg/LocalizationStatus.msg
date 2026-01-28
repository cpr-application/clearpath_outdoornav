# # Message definition containing localization status information for all sources of localization

std_msgs/Header header
# the localization accuracy as a percentage value (covariance?)
float32 accuracy # in development

# status information related to the GPS receiver units
GNSSStatus position_gnss
GNSSStatus heading_gnss
GNSSStatus base_station_gnss

bool dead_reckoning