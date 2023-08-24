# This message show planning status as well as path 
# header: ros header
# planning_status: planning_status,   
# planning status like bash access rwx, # 4 start_feasible, 2 goal_feasible, 1, path_feasible
# for example
#      0 means, start infeasible, goal infeasible, path infeasible
#      6 means, start and goal feasible but path infeasible
# poses: ROS poses array
# cannot visualize in rviz visualization
Header header
uint8 planning_status
geometry_msgs/PoseStamped[] poses