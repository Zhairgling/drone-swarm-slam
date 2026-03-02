# Incremental or full 3D map update from the SLAM pipeline
#
# Published on /slam/map_update by the SLAM node.
# Contains the new or changed points contributed by a drone.

std_msgs/Header header

# Drone that produced this update (0 = ground-station fused)
uint8 drone_id

# Whether this is a full map or incremental delta
uint8 update_type
uint8 UPDATE_FULL=0
uint8 UPDATE_INCREMENTAL=1

# 3D points added or updated in this batch
sensor_msgs/PointCloud2 points

# Origin of this update in the global map frame
geometry_msgs/Pose origin

# Voxel resolution in meters (ground station configurable)
float32 resolution
