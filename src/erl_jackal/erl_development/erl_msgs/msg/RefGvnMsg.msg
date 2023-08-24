# This represents reference governor state information
# rbt_pose2D: Robot position in 2D
# gvn_pose2D: Reference governor position in 2D
# lpg_pose2D: Local projected goal in 2D
# dQgO: Distance between governor and obstacle space in Q norm
# dIgO: Distance between governor and obstacle space in Eculidean norm
# dIcO: Distance between governor cone and obstacle space in Eculidean norm
# dIr: Distance between robot and obstacle space in Eculidean norm
# deltaE: Safety metric

Header header
geometry_msgs/Pose2D rbt_pose2D
geometry_msgs/Pose2D gvn_pose2D
geometry_msgs/Pose2D lpg_pose2D
float32 dQcO
float32 dIgO
float32 dIcO
float32 dIrO
float64 deltaE
int32 gvn_status
