#ifndef _SDF_MAP_H
#define _SDF_MAP_H

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
#include <queue>
#include <ros/ros.h>
#include <tuple>
#include <visualization_msgs/Marker.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>

#include <plan_env/raycast.h>

#include <fstream>
#include <memory>
#include <thread>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/duration.h>
#include <ros/ros.h>

//#include <erl_conversions/erl_msg_utils.h>
#include <erl_costmap_ros/erl_costmap.h>

#define logit(x) (log((x) / (1 - (x))))

using namespace std;

// constant parameters

struct MappingParameters {

    /* map properties */
    Eigen::Vector2d map_origin_, map_size_;
    Eigen::Vector2d map_min_boundary_, map_max_boundary_;// map range in pos
    Eigen::Vector2i map_voxel_num_;                      // map range in index
    Eigen::Vector2i map_min_idx_, map_max_idx_;
    Eigen::Vector2d local_update_range_;
    double resolution_, resolution_inv_;

    bool show_esdf_time_;
};

// intermediate mapping data for fusion, esdf

class SDFMap {
private:
    MappingParameters mp_;

    ros::NodeHandle node_;

    ros::Subscriber gridmap_sub_, odom_sub_, query_sub_;// GridMap Subscriber from a mapper.

    // publishers
    ros::Publisher planning_costmap_pub;// custom path publisher
    ros::Publisher rviz_costmap_pub;    // custom path publisher
    ros::Publisher esdf_pub_;           // publish esdf as pointcloud  binary_pub_
    ros::Publisher binary_pub_;         // publish binary map info as pointcloud

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

    // flags of map state
    std::atomic<bool> has_esdf_;
//    bool writing_esdf_temp_, has_odom_;

    // camera position and pose data
    Eigen::Vector3d robot_pos_;
    Eigen::Vector3d last_camera_pos_;
//    Eigen::Quaterniond camera_q_, last_camera_q_;
    Eigen::Matrix4d cam2body_;

    // esdf map data
    cv::Mat dist_map_buffer_;

    // computation time
    double esdf_time_, max_esdf_time_;


    // Publish esdf as point clould
    void publishESDF();

    // Publish binary map as point clould
    void publishBinary();

    // Odom callback
    void odomCallback(const nav_msgs::OdometryConstPtr &odom);

    // Gridmap callback
    void gridmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &gridmap_msg);

    // Query callback
    void queryCallback(const geometry_msgs::PoseStampedConstPtr &msg);

public:
    SDFMap() {
    }
    ~SDFMap() {
    }

    void overwriteDefaultSetting(ros::NodeHandle &nh) {
        // load gridmsg classification params
        nh.param("gridmsg_obstacle_lower_bound", p_gridmsg_obstacle_lower_bound, 80);
        nh.param("gridmsg_obstacle_upper_bound", p_gridmsg_obstacle_upper_bound, 100);
        nh.param("gridmsg_unknown", p_gridmsg_unknown, -1);

        // load params for costmap computation used for planning
        nh.param("costmap_planning_robot_inscribed_radius", p_costmap_planning_robot_inscribed_radius, 0.215);
        nh.param("costmap_planning_gamma", p_costmap_planning_gamma, 7.0);

        nh.param("costmap_planning_unknown", p_costmap_planning_unknown, 3);
        nh.param("costmap_planning_lethal", p_costmap_planning_lethal, 19);
        nh.param("costmap_planning_inscribed", p_costmap_planning_inscribed, 19);
        nh.param("costmap_planning_cutoff_cost", p_costmap_planning_cutoff_cost, 5);


        // load params for costmap display in rviz
        nh.param("costmap_visualization_unknown", p_costmap_visualization_unknown, -1);                // transparent? unknown explored region
        nh.param("costmap_visualization_lethal", p_costmap_visualization_lethal, 100);                 // purple,
        nh.param("costmap_visualization_inscribed", p_costmap_visualization_inscribed, 99);            // red
        nh.param("costmap_visualization_planning_cutoff", p_costmap_visualization_planning_cutoff, -3);// yellow
        nh.param("costmap_visualization_zero_cost", p_costmap_visualization_zero_cost, 0);             // green

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


    inline void posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id);
    inline void indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos);
    inline int toAddress(const Eigen::Vector2i &id);

    inline bool isInMap(const Eigen::Vector2d &pos);

    inline int getInflateOccupancy(Eigen::Vector2d pos);

    inline void boundIndex(Eigen::Vector2i &id);
    inline bool isUnknown(const Eigen::Vector2i &id);
    inline bool isKnownFree(const Eigen::Vector2d &pos);
    inline bool isKnownFree(const Eigen::Vector2i &id);

    // distance field management
    inline double getDistance(const Eigen::Vector2d &pos);
    inline double getDistance(const Eigen::Vector2i &id);
    double getDistWithGradTrilinear(Eigen::Vector2d pos, Eigen::Vector2d &grad);
    void getSurroundPts(const Eigen::Vector2d &pos, Eigen::Vector2d pts[2][2], Eigen::Vector2d &diff);

    // assembly costmap msg for planning
    inline nav_msgs::OccupancyGrid generatePlanningCostmapMsg(const nav_msgs::OccupancyGrid::ConstPtr &msg_in);

    // assembly costmap msg for rviz
    inline nav_msgs::OccupancyGrid generateRvizCostmapMsg(const nav_msgs::OccupancyGrid::ConstPtr &msg_in);


    void initMap(ros::NodeHandle &nh);

    void getRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size);
    double getResolution();
    Eigen::Vector2d getOrigin();
    int getVoxelNum();

    typedef std::shared_ptr<SDFMap> Ptr;

//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    //
};

/* ============================== definition of inline function ============================== */

inline int SDFMap::toAddress(const Eigen::Vector2i &id) {
    return id(0) + id(1) * mp_.map_voxel_num_(1);
}

inline void SDFMap::boundIndex(Eigen::Vector2i &id) {
    Eigen::Vector2i id1;
    id1(0) = max(min(id(0), mp_.map_voxel_num_(0) - 1), 0);
    id1(1) = max(min(id(1), mp_.map_voxel_num_(1) - 1), 0);
    id = id1;
}

inline double SDFMap::getDistance(const Eigen::Vector2d& pos) {
  Eigen::Vector2i id;
  posToIndex(pos, id);
  boundIndex(id);
//  cout << "num elem: " << dist_map_buffer_.elemSize() << std::endl;
//  cout << "size: " << dist_map_buffer_.size[0] << ", " << dist_map_buffer_.size[1] << std::endl;
//  cout << "channels: " << dist_map_buffer_.channels() << endl;

  return static_cast<double>(dist_map_buffer_.at<float>(static_cast<int>(toAddress(id))));
}

inline double SDFMap::getDistance(const Eigen::Vector2i &id) {
    Eigen::Vector2i id1 = id;
    boundIndex(id1);

    return static_cast<double>(dist_map_buffer_.at<float>(static_cast<int>(toAddress(id))));
}

inline bool SDFMap::isUnknown(const Eigen::Vector2i &id) {
    Eigen::Vector2i id1 = id;
    boundIndex(id1);

    return m_costmap->planning_costmap_data[toAddress(id1)] == m_costmap->setting->kCostUnknown;
}

inline bool SDFMap::isKnownFree(const Eigen::Vector2d &pos) {
    Eigen::Vector2i id1;
    posToIndex(pos, id1);
    boundIndex(id1);

    return !isUnknown(id1) && double(dist_map_buffer_.at<float>(toAddress(id1))) >= 0.05;
}

inline bool SDFMap::isKnownFree(const Eigen::Vector2i &id) {
    Eigen::Vector2i id1 = id;
    boundIndex(id1);

    return !isUnknown(id1) && double(dist_map_buffer_.at<float>(toAddress(id1))) >= 0.05;
}

inline double SDFMap::getDistWithGradTrilinear(Eigen::Vector2d pos, Eigen::Vector2d &grad) {
    if (!isInMap(pos)) {
        cout << "Not in map" << endl;
        grad.setZero();
        return 0;
    }

    /* use trilinear interpolation */
    Eigen::Vector2d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector2d::Ones();

    Eigen::Vector2i idx;
    posToIndex(pos_m, idx);

    Eigen::Vector2d idx_pos, diff;
    indexToPos(idx, idx_pos);

    diff = (pos - idx_pos) * mp_.resolution_inv_;

    double values[2][2];
    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 2; y++) {
            Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
            values[x][y] = getDistance(current_idx);
        }
    }

    double v0 = (1 - diff[0]) * values[0][0] + diff[0] * values[1][0];
    double v1 = (1 - diff[0]) * values[0][1] + diff[0] * values[1][1];
    double dist = (1 - diff[1]) * v0 + diff[1] * v1;

    grad[1] = (v1 - v0) * mp_.resolution_inv_;
    grad[0] = (1 - diff[1]) * (values[1][0] - values[0][0]);
    grad[0] += diff[1] * (values[1][1] - values[0][1]);

    grad[0] *= mp_.resolution_inv_;

    return dist;
}

inline int SDFMap::getInflateOccupancy(Eigen::Vector2d pos) {
    if (!isInMap(pos)) return -1;

    return int(getDistance(pos) >= 0.1 ? int(0) : int(1));
}

inline bool SDFMap::isInMap(const Eigen::Vector2d &pos) {
    if (pos(0) < mp_.map_min_boundary_(0) + 1e-4 || pos(1) < mp_.map_min_boundary_(1) + 1e-4) {
        // cout << "less than min range!" << endl;
        return false;
    }
    if (pos(0) > mp_.map_max_boundary_(0) - 1e-4 || pos(1) > mp_.map_max_boundary_(1) - 1e-4) {
        return false;
    }
    return true;
}

inline void SDFMap::posToIndex(const Eigen::Vector2d &pos, Eigen::Vector2i &id) {
    for (int i = 0; i < 2; ++i) {
        id(i) = floor((pos(i) - mp_.map_origin_(i)) * mp_.resolution_inv_);
        //    std::cout << "posToIndex Itr: " << i << "id :" << id(i) << "id type: " << typeid(id(i)).name() << std::endl;
    }
}

inline void SDFMap::indexToPos(const Eigen::Vector2i &id, Eigen::Vector2d &pos) {
    for (int i = 0; i < 2; ++i)
        pos(i) = (id(i) + 0.5) * mp_.resolution_ + mp_.map_origin_(i);
}

inline nav_msgs::OccupancyGrid SDFMap::generatePlanningCostmapMsg(const nav_msgs::OccupancyGrid::ConstPtr &msg_in) {
    nav_msgs::OccupancyGrid msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.info = msg_in->info;
    // be cautious, the data are all row-majored
    msg.data.assign(m_costmap->planning_costmap_data.begin(), m_costmap->planning_costmap_data.end());
    return msg;
}

inline nav_msgs::OccupancyGrid SDFMap::generateRvizCostmapMsg(const nav_msgs::OccupancyGrid::ConstPtr &msg_in) {
    nav_msgs::OccupancyGrid msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    msg.info = msg_in->info;
    // msg.info.map_load_time = ros::Time::now();
    // be cautious, the data are all row-majored
    msg.data.assign(m_costmap->rviz_costmap_data.begin(), m_costmap->rviz_costmap_data.end());
    return msg;
}

#endif
