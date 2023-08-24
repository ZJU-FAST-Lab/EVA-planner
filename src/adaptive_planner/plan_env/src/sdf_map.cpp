#include "plan_env/sdf_map.h"
#include <thread>
// #define current_img_ depth_image_[image_cnt_ & 1]
// #define last_img_ depth_image_[!(image_cnt_ & 1)]

void SDFMap::initMap(ros::NodeHandle &nh) {
    //----------------- ROS node handle ------------------
    node_ = nh;

    nh.param("show_esdf_time", mp_.show_esdf_time_, false);

    //----------------- Update status ------------------
//    has_odom_ = false;
    has_esdf_ = false;
//    writing_esdf_temp_ = true;

    esdf_time_ = 0.0;
    max_esdf_time_ = 0.0;

    //----------------- Ros interface ------------------
    // Incoming odom
    // odom_sub_ = nh.subscribe<nav_msgs::Odometry>("/sdf_map/odom", 1, &SDFMap::odomCallback, this);

    // Incoming gridmap sub
    gridmap_sub_ = nh.subscribe<nav_msgs::OccupancyGrid>("/map", 1, &SDFMap::gridmapCallback, this);

    // Query point sub
    query_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &SDFMap::queryCallback, this);

    // Visualization publishers
    planning_costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("/costmap", 1, 1);
    rviz_costmap_pub = nh.advertise<nav_msgs::OccupancyGrid>("costmap_rviz", 1, 1);
    esdf_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/esdf", 1);
    binary_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/sdf_map/binary", 1);

    // Waiting for grid map callback being called
    while (ros::ok() && (!has_esdf_)) {
//        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
}
void SDFMap::gridmapCallback(const nav_msgs::OccupancyGrid::ConstPtr &gridmap_msg) {
    /**
   * Callback for occupancy grid messages.
   * @param gridmap_msg The incoming occupancy grid message.
   */

    // init m_costmap class when first message received
    if (m_costmap == nullptr) {
        m_costmap = std::make_shared<erl::CostMap2D>(costmap_setting);
        overwriteDefaultSetting(node_);
    }

    //----------------- Map meta info ------------------
    // Retrieve map meta data
    if (!has_esdf_) {
        mp_.resolution_ = static_cast<double>(gridmap_msg->info.resolution);

        auto map_width_in_pixel = static_cast<double>(gridmap_msg->info.width);
        auto map_height_in_pixel = static_cast<double>(gridmap_msg->info.height);

        mp_.resolution_inv_ = 1.0 / mp_.resolution_;

        mp_.map_origin_ = Eigen::Vector2d(-1.0 * map_width_in_pixel * mp_.resolution_ / 2.0,
                                          -1.0 * map_height_in_pixel * mp_.resolution_ / 2.0);
        mp_.map_size_ = Eigen::Vector2d(map_width_in_pixel * mp_.resolution_, map_height_in_pixel * mp_.resolution_);

        mp_.map_voxel_num_ = Eigen::Vector2i(map_width_in_pixel, map_height_in_pixel);

        mp_.map_min_boundary_ = mp_.map_origin_;
        mp_.map_max_boundary_ = mp_.map_origin_ + mp_.map_size_;

        mp_.map_min_idx_ = Eigen::Vector2i::Zero();
        mp_.map_max_idx_ = mp_.map_voxel_num_ - Eigen::Vector2i::Ones();
    }

    //  cout << "map width: " << map_width_in_pixel << endl;
    //  cout << "map height: " << map_height_in_pixel << endl;
    //  cout << "map res: " << mp_.resolution_ << endl;
    //  cout << "map min: " << mp_.map_min_boundary_ << endl;
    //  cout << "map max: " << mp_.map_max_boundary_ << endl;

    ros::Time t1, t2;
    t1 = ros::Time::now();

    m_costmap->compute(gridmap_msg, dist_map_buffer_);
//    auto thread_id = std::this_thread::get_id();
//    std::cout << thread_id << ": " << __PRETTY_FUNCTION__ << ": " << &dist_map_buffer_ << '\t' << this << std::endl;
    //  dist_map_ = dist_map_buffer_.clone();

    t2 = ros::Time::now();

    esdf_time_ += (t2 - t1).toSec();
    max_esdf_time_ = max(max_esdf_time_, (t2 - t1).toSec());

    if (mp_.show_esdf_time_)
        ROS_WARN("ESDF: cur t = %lf, max t = %lf", (t2 - t1).toSec(), max_esdf_time_);

    // publish costmap msg for planning
    planning_costmap_pub.publish(SDFMap::generatePlanningCostmapMsg(gridmap_msg));

    // publish costmap msg for rviz
    rviz_costmap_pub.publish(SDFMap::generateRvizCostmapMsg(gridmap_msg));

    publishBinary();
    publishESDF();

    has_esdf_ = true;
//    std::cout << __PRETTY_FUNCTION__ << ": done" << std::endl;
}

void SDFMap::odomCallback(const nav_msgs::OdometryConstPtr &odom) {
    robot_pos_(0) = odom->pose.pose.position.x;
    robot_pos_(1) = odom->pose.pose.position.y;

    Eigen::Quaterniond body_q = Eigen::Quaterniond(odom->pose.pose.orientation.w,
                                                   odom->pose.pose.orientation.x,
                                                   odom->pose.pose.orientation.y,
                                                   odom->pose.pose.orientation.z);

    robot_pos_(2) = body_q.toRotationMatrix().eulerAngles(0, 1, 2).z();

    robot_pos_(2) = odom->pose.pose.position.z;

//    has_odom_ = true;
}

void SDFMap::publishESDF() {

    if (has_esdf_) {
        double dist;
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::PointXYZI pt;

        const double min_dist = 0.0;
        const double max_dist = 5.0;

        // Eigen::Vector3i min_cut = local_bound_min_ -
        //     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
        // Eigen::Vector3i max_cut = local_bound_max_ +
        //     Eigen::Vector3i(mp_.local_map_margin_, mp_.local_map_margin_, mp_.local_map_margin_);
        Eigen::Vector2i min_cut = mp_.map_min_idx_;
        Eigen::Vector2i max_cut = mp_.map_max_idx_;
        boundIndex(min_cut);
        boundIndex(max_cut);

        for (int x = min_cut(0); x <= max_cut(0); ++x)
            for (int y = min_cut(1); y <= max_cut(1); ++y) {

                Eigen::Vector2d pos;
                indexToPos(Eigen::Vector2i(x, y), pos);

                dist = getDistance(pos);
                dist = min(dist, max_dist);
                dist = max(dist, min_dist);

                pt.x = static_cast<double>(pos(0));
                pt.y = static_cast<double>(pos(1));
                pt.z = 0.0;
                pt.intensity = static_cast<double>(dist - min_dist) / (max_dist - min_dist);
                cloud.push_back(pt);
            }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = "map";
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);

        esdf_pub_.publish(cloud_msg);
    } else {
        return;
    }
}

void SDFMap::publishBinary() {

    if (has_esdf_) {
        double binary_data;
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::PointXYZI pt;

        Eigen::Vector2i min_cut = mp_.map_min_idx_;
        Eigen::Vector2i max_cut = mp_.map_max_idx_;
        boundIndex(min_cut);
        boundIndex(max_cut);

        for (int x = min_cut(0); x <= max_cut(0); ++x)
            for (int y = min_cut(1); y <= max_cut(1); ++y) {

                Eigen::Vector2d pos;
                indexToPos(Eigen::Vector2i(x, y), pos);

                binary_data = isKnownFree(pos);

                pt.x = static_cast<double>(pos(0));
                pt.y = static_cast<double>(pos(1));
                pt.z = 0.0;
                if (binary_data == true) {
                    pt.intensity = 5.0;
                } else {
                    pt.intensity = 0.0;
                }
                cloud.push_back(pt);
            }

        cloud.width = cloud.points.size();
        cloud.height = 1;
        cloud.is_dense = true;
        cloud.header.frame_id = "map";
        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(cloud, cloud_msg);

        binary_pub_.publish(cloud_msg);
    } else {
        return;
    }
}

void SDFMap::queryCallback(const geometry_msgs::PoseStampedConstPtr &msg) {

    // Obtain query point
    Eigen::Vector2d point, grad;
    point = Eigen::Vector2d(msg->pose.position.x, msg->pose.position.y);

    cout << point << endl;

    if (has_esdf_) {
        double dist = getDistWithGradTrilinear(point, grad);
        ROS_WARN("[Interp] ESDF = %.2f, grad = [%.2f, %.2f]", dist, grad[0], grad[1]);
        ROS_WARN("ESDF = %.2f, Free = %d", getDistance(point), isKnownFree(point));
    }
}

/* -------------------------------------------------------------------------------------------------------------- */

double SDFMap::getResolution() {
    return mp_.resolution_;
}

Eigen::Vector2d SDFMap::getOrigin() {
    return mp_.map_origin_;
}

int SDFMap::getVoxelNum() {
    return mp_.map_voxel_num_[0] * mp_.map_voxel_num_[1];
}

void SDFMap::getRegion(Eigen::Vector2d &ori, Eigen::Vector2d &size) {
    ori = mp_.map_origin_, size = mp_.map_size_;
}

void SDFMap::getSurroundPts(const Eigen::Vector2d &pos, Eigen::Vector2d pts[2][2],
                            Eigen::Vector2d &diff) {
    if (!isInMap(pos)) {
        // cout << "pos invalid for interpolation." << endl;
    }

    /* interpolation position */
    Eigen::Vector2d pos_m = pos - 0.5 * mp_.resolution_ * Eigen::Vector2d::Ones();
    Eigen::Vector2i idx;
    Eigen::Vector2d idx_pos;

    posToIndex(pos_m, idx);
    indexToPos(idx, idx_pos);
    diff = (pos - idx_pos) * mp_.resolution_inv_;

    for (int x = 0; x < 2; x++) {
        for (int y = 0; y < 2; y++) {
            Eigen::Vector2i current_idx = idx + Eigen::Vector2i(x, y);
            Eigen::Vector2d current_pos;
            indexToPos(current_idx, current_pos);
            pts[x][y] = current_pos;
        }
    }
}

//double SDFMap::getDistance(const Eigen::Vector2d& pos) {
//    Eigen::Vector2i id;
//    posToIndex(pos, id);
//    boundIndex(id);
////    cout << "num elem: " << dist_map_buffer_.elemSize() << std::endl;
////    cout << "size: " << dist_map_buffer_.size[0] << ", " << dist_map_buffer_.size[1] << std::endl;
////    cout << "channels: " << dist_map_buffer_.channels() << endl;
//
//    return static_cast<double>(dist_map_buffer_.at<float>(static_cast<int>(toAddress(id))));
//}
//
//double SDFMap::getDistance(const Eigen::Vector2i &id) {
//    Eigen::Vector2i id1 = id;
//    boundIndex(id1);
//
//    return static_cast<double>(dist_map_buffer_.at<float>(static_cast<int>(toAddress(id))));
//}

int main(int argc, char** argv) {
  ros::init(argc, argv, "adaptive_planner_node");
  ros::NodeHandle nh("~");

  ros::AsyncSpinner spinner(18);
  spinner.start();

  SDFMap map;
  map.initMap(nh);

  ros::waitForShutdown();

  return 0;
}
