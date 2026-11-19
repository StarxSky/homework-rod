# A single obstacle detected from a 2D lidar scan.

std_msgs/Header header

# Sequential identifier assigned to this obstacle in the current scan.
int32 id

# Centroid of the obstacle in the output frame (metres).
geometry_msgs/Point position

# Planar distance from the sensor to the centroid (metres).
float32 distance

# Bearing from the sensor to the centroid (radians, -pi..pi).
float32 angle

# Approximate extent of the obstacle cluster (metres).
float32 width

# Number of lidar points that belong to this obstacle.
uint32 point_count
