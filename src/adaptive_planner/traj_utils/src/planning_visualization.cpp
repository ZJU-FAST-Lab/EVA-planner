#include <traj_utils/planning_visualization.h>

namespace adaptive_planner{

PlanningVisualization::PlanningVisualization(ros::NodeHandle& nh){
    node = nh;

    path_pub                = node.advertise<visualization_msgs::Marker>("/planning_vis/path",20);                    // 0
    pubs_.push_back(path_pub);
    low_mpc_traj_pub_       = node.advertise<visualization_msgs::Marker>("/planning_vis/low_mpc_traj", 20);           // 1
    pubs_.push_back(low_mpc_traj_pub_);
    local_goal_pub_         = node.advertise<visualization_msgs::Marker>("/planning_vis/local_goal", 20);             // 2
    pubs_.push_back(local_goal_pub_);
    high_mpc_traj_pub_      = node.advertise<visualization_msgs::Marker>("/planning_vis/high_mpcc_traj", 20);         // 3
    pubs_.push_back(high_mpc_traj_pub_);
    high_mpc_traj_ref_pub_  = node.advertise<visualization_msgs::Marker>("/planning_vis/high_mpcc_ref_traj", 20);     // 4
    pubs_.push_back(high_mpc_traj_ref_pub_);
    pos_cmd_pub_            = node.advertise<visualization_msgs::Marker>("/planning_vis/pos_cmd_vis",20);             // 5
    pubs_.push_back(pos_cmd_pub_);
    exected_traj_pub_       = node.advertise<visualization_msgs::Marker>("/travel_traj", 20);                         // 6
    pubs_.push_back(exected_traj_pub_);
}

void PlanningVisualization::displaySphereList(const vector<Eigen::Vector3d>& list, double resolution,
                                              const Eigen::Vector4d& color, int id, int pub_id) {
    visualization_msgs::Marker mk;
    mk.header.frame_id = "world";
    mk.header.stamp    = ros::Time::now();
    mk.type            = visualization_msgs::Marker::SPHERE_LIST;
    mk.action          = visualization_msgs::Marker::DELETE;
    mk.id              = id;
    pubs_[pub_id].publish(mk);

    mk.action             = visualization_msgs::Marker::ADD;
    mk.pose.orientation.x = 0.0;
    mk.pose.orientation.y = 0.0;
    mk.pose.orientation.z = 0.0;
    mk.pose.orientation.w = 1.0;

    mk.color.r = color(0);
    mk.color.g = color(1);
    mk.color.b = color(2);
    mk.color.a = color(3);

    mk.scale.x = resolution;
    mk.scale.y = resolution;
    mk.scale.z = resolution;

    geometry_msgs::Point pt;
    for (int i = 0; i < int(list.size()); i++) {
        pt.x = list[i](0);
        pt.y = list[i](1);
        pt.z = list[i](2);
        mk.points.push_back(pt);
    }
    pubs_[pub_id].publish(mk);
    ros::Duration(0.001).sleep();
}

void PlanningVisualization::displayCubeList(const vector<Eigen::Vector3d>& list, double resolution,
                                            const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::CUBE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id].publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  mk.scale.x = resolution;
  mk.scale.y = resolution;
  mk.scale.z = resolution;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); i++) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);

  ros::Duration(0.001).sleep();
}

void PlanningVisualization::displayLineList(const vector<Eigen::Vector3d>& list1,
                                            const vector<Eigen::Vector3d>& list2, double line_width,
                                            const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::LINE_LIST;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id].publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = line_width;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list1.size()); ++i) {
    pt.x = list1[i](0);
    pt.y = list1[i](1);
    pt.z = list1[i](2);
    mk.points.push_back(pt);

    pt.x = list2[i](0);
    pt.y = list2[i](1);
    pt.z = list2[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);

  ros::Duration(0.001).sleep();
}

void PlanningVisualization::displayLineStrip(const vector<Eigen::Vector3d>& list, double line_width,
                                            const Eigen::Vector4d& color, int id, int pub_id) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp    = ros::Time::now();
  mk.type            = visualization_msgs::Marker::LINE_STRIP;
  mk.action          = visualization_msgs::Marker::DELETE;
  mk.id              = id;
  pubs_[pub_id].publish(mk);

  mk.action             = visualization_msgs::Marker::ADD;
  mk.pose.orientation.x = 0.0;
  mk.pose.orientation.y = 0.0;
  mk.pose.orientation.z = 0.0;
  mk.pose.orientation.w = 1.0;

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);
  mk.scale.x = line_width;

  geometry_msgs::Point pt;
  for (int i = 0; i < int(list.size()); ++i) {
    pt.x = list[i](0);
    pt.y = list[i](1);
    pt.z = list[i](2);
    mk.points.push_back(pt);
  }
  pubs_[pub_id].publish(mk);

  ros::Duration(0.001).sleep();
}

void PlanningVisualization::drawPosCmd(const Eigen::Vector3d& pos, const Eigen::Vector3d& vec, const int& id,
             const Eigen::Vector4d& color) {
  visualization_msgs::Marker mk;
  mk.header.frame_id = "world";
  mk.header.stamp = ros::Time::now();
  mk.id = id;
  mk.type = visualization_msgs::Marker::ARROW;
  mk.action = visualization_msgs::Marker::ADD;

  mk.pose.orientation.w = 1.0;
  mk.scale.x = 0.1;
  mk.scale.y = 0.2;
  mk.scale.z = 0.3;

  geometry_msgs::Point pt;
  pt.x = pos(0);
  pt.y = pos(1);
  pt.z = pos(2);
  mk.points.push_back(pt);

  pt.x = pos(0) + vec(0);
  pt.y = pos(1) + vec(1);
  pt.z = pos(2) + vec(2);
  mk.points.push_back(pt);

  mk.color.r = color(0);
  mk.color.g = color(1);
  mk.color.b = color(2);
  mk.color.a = color(3);

  pubs_[5].publish(mk);
}

void PlanningVisualization::drawGoal(Eigen::Vector3d goal, double resolution,
                                     const Eigen::Vector4d& color, int id) {
  vector<Eigen::Vector3d> goal_vec = { goal };
  displaySphereList(goal_vec, resolution, color, 1);
}

void PlanningVisualization::drawPath(const vector<Eigen::Vector3d>& path, double line_width,
                                     const Eigen::Vector4d& color, int id){
  displayLineStrip(path, line_width, color, 1);
}

void PlanningVisualization::drawLocalGoal(Eigen::Vector3d local_goal, double resolution,
                                     const Eigen::Vector4d& color, int id) {
  vector<Eigen::Vector3d> local_goal_vec = { local_goal };
  displaySphereList(local_goal_vec, resolution, color, 1, 2);
}

void PlanningVisualization::drawLowMpcTraj(const vector<Eigen::Vector3d>& traj, double resolution,
                                     const Eigen::Vector4d& color, int id) {
  displaySphereList(traj, resolution, color, 1, 1);
}

void PlanningVisualization::drawHighMpccTraj(const vector<Eigen::Vector3d>& traj, double resolution,
                                     const Eigen::Vector4d& color, int id) {
  displaySphereList(traj, resolution, color, 1, 3);
}

void PlanningVisualization::drawHighMpccRefTraj(const vector<Eigen::Vector3d>& traj, double resolution,
                                     const Eigen::Vector4d& color, int id) {
  displaySphereList(traj, resolution, color, 1, 4);
}

void drawExectedTraj(Eigen::Vector3d pos1, Eigen::Vector3d pos2, double resolution,
                     const Eigen::Vector4d& color, int id){
  // id = 6;
  
}

} // namespace adaptive_planner