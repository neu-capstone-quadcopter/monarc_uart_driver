# Define the goal
# Keep this in sync with monarcpb!
int8 TAKEOFF = 0
int8 LAND = 1
int8 FORWARD = 2
int8 UP = 3
int8 DOWN = 4
int8 TURN_RIGHT_90 = 5
int8 TURN_LEFT_90 = 6
int8 NAVIGATE_TO_GOAL = 7

int8 command_number
sensor_msgs/NavSatFix command_location
