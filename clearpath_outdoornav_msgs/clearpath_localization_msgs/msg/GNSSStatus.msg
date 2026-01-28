# Message definition containing the GNSS receiver status information.

uint8 POSITION_RECEIVER = 0
uint8 HEADING_RECEIVER = 1
uint8 BASE_STATION_RECEIVER = 2
uint8 receiver_type

uint8 POSITION_NO_FIX = 0
uint8 POSITION_SPP = 1
uint8 POSITION_SBAS = 2
uint8 POSITION_RTK_FLOAT = 3
uint8 POSITION_RTK_FIXED = 4
uint8 position_fix_type

uint8 HEADING_NOT_APPLICABLE = 0
uint8 HEADING_NO_SOLUTION = 1
uint8 HEADING_RTK_FLOAT = 2
uint8 HEADING_RTK_FIXED = 3
uint8 heading_fix_type

uint8 num_connected_sats	# Number of satellites connected to the antenna/receiver
uint8 num_sats_solution   # Number of sats used in solution
float32[] cn0			        # The carrier-to-noise ratio of each connected satellite
float64 correction_age    # Age of RTK corrections. -1 indicates none received
