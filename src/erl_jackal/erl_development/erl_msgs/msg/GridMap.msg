std_msgs/Header header

float32[] resolution  # Resolution of each dimension (X, Y, Z)

uint32[] dim  # Dimension of the GridMap (d_x  * d_y * d_z)

float32[] origin  # Origin as an array of float.

bool rowmajor # is the map stored in rowmajor or colmajor order

# Subscript (xi, yi, zi) indices probability data[xi + dim.x * yi + zi * dim.y * dim.z]
int8[] data

