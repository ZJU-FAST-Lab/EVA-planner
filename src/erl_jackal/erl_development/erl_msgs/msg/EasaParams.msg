# This represents parameters for EASA safety metric
# rbt_pose2D: Robot position in 2D
# dIrO: Distance between robot and obstacle space in Euclidean norm
# delta_c: Gradient vector of TSDF
# rbt_velocity: Robot velocity
# beta: Risk factor
# J_c: EASA distance penalty
# f_e: EASA safety penalty

Header header
geometry_msgs/Pose2D rbt_pose2D
float32 dIrO
geometry_msgs/Pose2D delta_c
geometry_msgs/Pose2D rbt_velocity
float64 beta
float64 J_c
float64 f_e
