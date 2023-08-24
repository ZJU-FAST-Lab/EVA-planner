#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>


std::string TF_link_parent_frame_id;
std::string TF_link_child_frame_id;

std::string local_map_frame_id;
std::string global_map_frame_id;

geometry_msgs::TransformStamped mapTransform;
ros::Publisher pub;

/**
This Node listens to an odometry topic in the global frame, and publishes the correct TF
 and odometry topic in the local frame.
**/

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  static tf2_ros::TransformBroadcaster odom_br;
  geometry_msgs::TransformStamped transformStamped;

  // Output the mapTransform, Transform from global to local map,
  ROS_WARN_ONCE("[link_tf] mapTransform from local [%s] to global [%s]: X: %f, Y: %f, Z: %f",
    local_map_frame_id.c_str(),
    global_map_frame_id.c_str(),
    mapTransform.transform.translation.x,
    mapTransform.transform.translation.y,
    mapTransform.transform.translation.z);

  // Apply Transform. (Input = pose, Output = pose_out, Transform = mapTransform)
  geometry_msgs::Pose pose_out;
  auto pose_in = msg->pose.pose;
  tf2::doTransform(pose_in, pose_out, mapTransform);

  // Publish TF
  transformStamped.header = msg->header;
  transformStamped.header.frame_id = TF_link_parent_frame_id;
  transformStamped.child_frame_id = TF_link_child_frame_id;
  transformStamped.transform.translation.x = pose_out.position.x;
  transformStamped.transform.translation.y = pose_out.position.y;
  transformStamped.transform.translation.z = pose_out.position.z;
  transformStamped.transform.rotation.x = pose_out.orientation.x;
  transformStamped.transform.rotation.y = pose_out.orientation.y;
  transformStamped.transform.rotation.z = pose_out.orientation.z;
  transformStamped.transform.rotation.w = pose_out.orientation.w;

  odom_br.sendTransform(transformStamped);
  ROS_WARN_ONCE("publish tf from [%s] to [%s]",
                transformStamped.header.frame_id.c_str(),
                transformStamped.child_frame_id.c_str());

  // Publish Ground Truth Odometry in local map
  nav_msgs::Odometry gt_odom;
  gt_odom.header.frame_id = TF_link_parent_frame_id;
  gt_odom.child_frame_id = TF_link_child_frame_id;
  gt_odom.pose.pose.position.x = pose_out.position.x;
  gt_odom.pose.pose.position.y = pose_out.position.y;
  gt_odom.pose.pose.position.z = pose_out.position.z;
  gt_odom.pose.pose.orientation.x = pose_out.orientation.x;
  gt_odom.pose.pose.orientation.y = pose_out.orientation.y;
  gt_odom.pose.pose.orientation.z = pose_out.orientation.z;
  gt_odom.pose.pose.orientation.w = pose_out.orientation.w;

  pub.publish(gt_odom);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "link_tf");
  ros::NodeHandle nh("~");

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  nh.param("global_map_frame_id", global_map_frame_id, std::string("world"));
  nh.param("local_map_frame_id", local_map_frame_id, std::string("map"));
  nh.param("TF_link_child_frame_id", TF_link_child_frame_id, std::string("base_link"));

  TF_link_parent_frame_id = local_map_frame_id;
  
  ROS_WARN("[link_tf]");
  ROS_WARN_STREAM("TF_link_parent_frame_id: " << TF_link_parent_frame_id);
  ROS_WARN_STREAM("TF_link_child_frame_id: " << TF_link_child_frame_id);
  ROS_WARN_STREAM("global_map_frame_id: " << global_map_frame_id);
  ROS_WARN_STREAM("local_map_frame_id: " << local_map_frame_id);
  
  ros::Subscriber sub = nh.subscribe("input", 10, odomCallback, ros::TransportHints().tcpNoDelay());

  pub = nh.advertise<nav_msgs::Odometry>("output", 10);

  ros::Rate loop_rate(50);

  while(ros::ok())
  {
    try {
      // Get inverse from local map to global map
      mapTransform = tfBuffer.lookupTransform(local_map_frame_id, global_map_frame_id,ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("[p3d_link_tf_complex] %s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
};
