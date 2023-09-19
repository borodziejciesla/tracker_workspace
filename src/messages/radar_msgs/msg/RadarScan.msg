# Define radar scan

std_msgs/Header header      # [N/A]

float64 timestamp           # [s]

uint8 radar_id              # [-]
SensorOrigin sensor_origin  # [N/A]

uint16 detections_number    # [-]
Detection[] detections      # [N/A]