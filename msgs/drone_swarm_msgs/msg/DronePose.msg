# Estimated pose of a drone with confidence and source metadata
#
# Published on /drone_N/pose/estimated by the ground station
# pose estimator. Consumed by the mission controller and sent
# back to the drone for position corrections.

std_msgs/Header header

# Drone identifier (matches N in /drone_N/ topics)
uint8 drone_id

# Estimated 6-DoF pose in the SLAM map frame
geometry_msgs/Pose pose

# Estimated linear and angular velocity
geometry_msgs/Twist velocity

# Confidence score [0.0, 1.0] — 0 = lost, 1 = high confidence
float32 confidence

# Source of this pose estimate
uint8 source
uint8 SOURCE_SLAM=0
uint8 SOURCE_GPS=1
uint8 SOURCE_FUSED=2
