#include "geometry_msgs/msg/vector3.h"

string family
int32 id
int32 hamming
float32 goodness
float32 decision_margin
geometry_msgs/Vector3 pose                    # tag pose in camera frame (x,y,z)[m]
geometry_msgs/Vector3 rotation                # tag rotation in camera frame (x,y,z)[rad]
Point centre                    # centre in (x,y) pixel coordinates
Point[4] corners                # corners of tag ((x1,y1),(x2,y2),...)
float64[9] homography           # 3x3 row-major homography matrix
