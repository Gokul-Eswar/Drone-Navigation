# Custom message to aggregate all relevant drone state information

std_msgs/Header header

# Position and Orientation
geometry_msgs/Pose pose

# Velocity
geometry_msgs/Twist velocity

# Battery State
float32 battery_percentage

# MAVROS State
bool armed
bool connected
string flight_mode
