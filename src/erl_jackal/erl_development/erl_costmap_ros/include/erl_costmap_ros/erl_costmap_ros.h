#ifndef ERL_COSTMAP_ROS_H
#define ERL_COSTMAP_ROS_H

#include <erl_costmap_ros/erl_costmap.h>
//#include <erl_conversions/erl_msg_utils.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <fstream>
#include <memory>

/**
 * @brief ErlCostmapROS The ErlCostmapROS class is responsible for reading in all configuration
 * parameters for costmap2d needed by erl_astar_costmap2d, as well as managing necessary
 * subscribers and publishers to generate costmap msgs (wrapped as occupancy grid message) for planning and rviz visualiztion.
 */

class ErlCostmapROS {
 public:
  /**
   * @brief Constructs the ErlCostmapROS.
   * @param nh The ROS nodehandle.
   */

  /**
   * @brief Read parameters from ROS parameter server and overwrite default setting.
   */

  void overwriteDefaultSetting() {
    // load gridmsg classification params    
    nh.param("gridmsg_obstacle_lower_bound", p_gridmsg_obstacle_lower_bound, 80);
    nh.param("gridmsg_obstacle_upper_bound", p_gridmsg_obstacle_upper_bound, 100);
    nh.param("gridmsg_unknown", p_gridmsg_unknown, -1);
    
    // load params for costmap computation used for planning
    nh.param("costmap_planning_robot_inscribed_radius", p_costmap_planning_robot_inscribed_radius, 0.215);
    nh.param("costmap_planning_gamma", p_costmap_planning_gamma, 7.0);
    
    nh.param("costmap_planning_unknown", p_costmap_planning_unknown, 3);
    nh.param("costmap_planning_lethal", p_costmap_planning_lethal, 19);
    nh.param("costmap_planning_inscribed", p_costmap_planning_inscribed, 18);
    nh.param("costmap_planning_cutoff_cost", p_costmap_planning_cutoff_cost, 6);


    // load params for costmap display in rviz
    nh.param("costmap_visualization_unknown", p_costmap_visualization_unknown, -1); // transparent? unknown explored region
    nh.param("costmap_visualization_lethal", p_costmap_visualization_lethal, 100); // purple, 
    nh.param("costmap_visualization_inscribed", p_costmap_visualization_inscribed, 90); // red
    nh.param("costmap_visualization_planning_cutoff", p_costmap_visualization_planning_cutoff, -3); // yellow
    nh.param("costmap_visualization_zero_cost", p_costmap_visualization_zero_cost, 110); // green
    
    // set flag for param reading
    // ros_param_received = true;

    costmap_setting->kUnknown = static_cast<int8_t>(p_gridmsg_unknown);
    costmap_setting->kObsLb = static_cast<int8_t>(p_gridmsg_obstacle_lower_bound);
    costmap_setting->kObsUb = static_cast<int8_t>(p_gridmsg_obstacle_upper_bound);
    
    costmap_setting->kCostGamma = p_costmap_planning_gamma;
    costmap_setting->kCostRadius1 = p_costmap_planning_robot_inscribed_radius;
    costmap_setting->kCostUnknown = static_cast<int8_t>(p_costmap_planning_unknown);
    costmap_setting->kCostLethal = static_cast<int8_t>(p_costmap_planning_lethal);
    costmap_setting->kCostInscribed = static_cast<int8_t>(p_costmap_planning_inscribed);
    costmap_setting->kCostCutoff = static_cast<int8_t>(p_costmap_planning_cutoff_cost);

    costmap_setting->kVisUnknown = static_cast<int8_t>(p_costmap_visualization_unknown);
    costmap_setting->kVisLethal = static_cast<int8_t>(p_costmap_visualization_lethal);
    costmap_setting->kVisInscribed = static_cast<int8_t>(p_costmap_visualization_inscribed);
    costmap_setting->kVisCutoff = static_cast<int8_t>(p_costmap_visualization_planning_cutoff);
    costmap_setting->kVisZero = static_cast<int8_t>(p_costmap_visualization_zero_cost);
    

    ROS_INFO("ErlCostmap kUnknown: %d", costmap_setting->kUnknown);
    ROS_INFO("ErlCostmap kObsLb: %d", costmap_setting->kObsLb);
    ROS_INFO("ErlCostmap kObsUb: %d\n", costmap_setting->kObsUb);
    
    
    ROS_INFO("ErlCostmap kCostGamma: %.2f", costmap_setting->kCostGamma);
    ROS_INFO("ErlCostmap kCostRadius1: %.2f", costmap_setting->kCostRadius1);
    ROS_INFO("ErlCostmap kCostUnknown: %d", costmap_setting->kCostUnknown);
    ROS_INFO("ErlCostmap kCostLethal: %d", costmap_setting->kCostLethal);
    ROS_INFO("ErlCostmap kCostInscribed: %d", costmap_setting->kCostInscribed);
    ROS_INFO("ErlCostmap kCostCutoff: %d\n", costmap_setting->kCostCutoff);

    ROS_INFO("ErlCostmap kVisUnknown: %d", costmap_setting->kVisUnknown);
    ROS_INFO("ErlCostmap kVisLethal: %d", costmap_setting->kVisLethal);
    ROS_INFO("ErlCostmap kVisInscribed: %d", costmap_setting->kVisInscribed);
    ROS_INFO("ErlCostmap kVisCutoff: %d", costmap_setting->kVisCutoff);
    ROS_INFO("ErlCostmap kVisZero: %d\n", costmap_setting->kVisZero);

  }

  explicit ErlCostmapROS(ros::NodeHandle &nh) : nh(nh) {
    
    // Setup Publishers and then Subscribers
    planning_costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("/costmap", 1, 1);
    rviz_costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("costmap_rviz", 1, 1);
    gridmap_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 2, &ErlCostmapROS::gridmapCallback, this);
  }  // end of constructor


// assembly costmap msg for planning
inline nav_msgs::OccupancyGrid generatePlanningCostmapMsg(const nav_msgs::OccupancyGrid::ConstPtr &msg_in) {
    nav_msgs::OccupancyGrid msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.info = msg_in->info;
    // be cautious, the data are all row-majored
    msg.data.assign(m_costmap->planning_costmap_data.begin(), m_costmap->planning_costmap_data.end());
    return msg;
}

// assembly costmap msg for rviz
inline nav_msgs::OccupancyGrid generateRvizCostmapMsg(const nav_msgs::OccupancyGrid::ConstPtr &msg_in) {
    nav_msgs::OccupancyGrid msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.info = msg_in->info;
    // msg.info.map_load_time = ros::Time::now();
    // be cautious, the data are all row-majored
    msg.data.assign(m_costmap->rviz_costmap_data.begin(), m_costmap->rviz_costmap_data.end());
    return msg;
}


 private:
  // ----------------------- ROS -------------------------------
  ros::NodeHandle nh;
  // subscribers
  ros::Subscriber gridmap_sub;        // GridMap Subscriber from a mapper.

  // publishers
  ros::Publisher planning_costmap_pub;  // custom path publisher
  ros::Publisher rviz_costmap_pub;  // custom path publisher

  // costmap setting and costmap generator
  std::shared_ptr<erl::CostMap2D::Setting> costmap_setting = std::make_shared<erl::CostMap2D::Setting>();
  std::shared_ptr<erl::CostMap2D> m_costmap = nullptr;


  // ----------------------- Parameters --------------------------

  // params for occupancy grid msg (hector map) to value classification
  int p_gridmsg_obstacle_lower_bound;
  int p_gridmsg_obstacle_upper_bound;
  int p_gridmsg_unknown;

  // params for costmap computation used for planning
  double p_costmap_planning_gamma;
  double p_costmap_planning_robot_inscribed_radius;
  int p_costmap_planning_unknown;
  int p_costmap_planning_lethal;
  int p_costmap_planning_inscribed;
  int p_costmap_planning_cutoff_cost;


  // params for costmap computation used for ROS rviz visualization
  int p_costmap_visualization_unknown;
  int p_costmap_visualization_lethal;
  int p_costmap_visualization_inscribed;
  int p_costmap_visualization_planning_cutoff;
  int p_costmap_visualization_zero_cost;


  // // ----------------------- Status Flag --------------------------
  
  // // read param from node parameter server
  // bool ros_param_received{false};
  // // overwrite to class setting
  // bool ros_param_loaded{false};
  
  // ----------------------- FUNCTIONS --------------------------

  /**
   * Callback for occupancy grid messages. 
   * @param gridmap_msg The incoming occupancy grid message.
   */
  void gridmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &gridmap_msg) {
    // init m_costmap class when first message received
    if (m_costmap == nullptr) {
      m_costmap = std::make_shared<erl::CostMap2D>(costmap_setting);
      overwriteDefaultSetting();
    }
//    m_costmap->compute(gridmap_msg);

    // publish costmap msg for planning
    planning_costmap_pub.publish(generatePlanningCostmapMsg(gridmap_msg));
    
    // publish costmap msg for rviz 
    rviz_costmap_pub.publish(generateRvizCostmapMsg(gridmap_msg));
  }
}; // end of ErlCostmapROS class

#endif
