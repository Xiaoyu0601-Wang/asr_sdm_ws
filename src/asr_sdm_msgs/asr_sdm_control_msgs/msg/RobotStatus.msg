# RobotStatus.msg
std_msgs/Header header

# Status of the robot
bool is_operational       # True if the robot is operational, false otherwise
bool is_charging          # True if the robot is currently charging
float32 battery_level     # Battery level as a percentage (0.0 to 100.0)

# Robot's current mode
string current_mode       # Current mode of the robot (e.g., "IDLE", "AUTONOMOUS", "MANUAL")

# Robot's position
float32 position_x        # X-coordinate of the robot's position
float32 position_y        # Y-coordinate of the robot's position
float32 position_z        # Z-coordinate of the robot's position

# Robot's orientation
float32 orientation_roll  # Roll angle in radians
float32 orientation_pitch # Pitch angle in radians
float32 orientation_yaw   # Yaw angle in radians

# Additional information
string error_message      # Error message if the robot is in an error state