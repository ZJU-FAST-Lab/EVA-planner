# This represents cone controller status information
# z: Current robot state
# z_dsr: Desired robot state
# e_pose2d_raw: z_dsr - z
# err_angle: Angle difference between z_dsr and z
# err_angle_norm: Norm of err_angle
# err_angle_deg: err_angle in degree
# err_dist: Distance between z_dsr and z
# e_proj_u1: Distance error projected to robot heading 
# e_proj_u2: Distance error projected to robot heading  + 90 CCW
# err_heading: atan2(e_proj_u2, e_proj_u1) 
# err_heading_norm: norm of err_heading
# cone_controller_status: Current active controller
# v_min: The min linear speed of control output
# v_max: The max linear speed of control output
# w_min: The min angular speed of control output
# w_max: The max angular speed of control output
# sddm_boost: The linear control gain boost from directional metric 

Header header
geometry_msgs/Pose2D z
geometry_msgs/Pose2D z_dsr
geometry_msgs/Pose2D e_pose2d_raw
float32 err_angle
float32 err_angle_norm
float32 err_angle_deg
float64 err_dist
float32 e_proj_u1
float32 e_proj_u2
float32 err_heading
float32 err_heading_norm
int8 cone_controller_status
float32 v_min
float32 v_max
float32 w_min
float32 w_max
float32 sddm_boost
float32 easa_boost