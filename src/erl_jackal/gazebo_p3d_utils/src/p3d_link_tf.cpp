#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <string.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

/**
This Node listens to an odometry topic in the map frame, and publishes
correct transformation to complete TF tree.
Frames must be consistent with p3d plugin insertion in robot description.
e.g., linking ground truth state (map frame) and robot body frame (base_link).

e.g. jackal_description/urdf/jackal.gazebo

  <gazebo>
    <plugin name="p3d_base_controller" filename="libgazebo_ros_p3d.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>50.0</updateRate>
      <bodyName>base_link</bodyName>
      <topicName>ground_truth/state</topicName>
      <gaussianNoise>0.0</gaussianNoise>
      <frameName>map</frameName>
      <xyzOffset>0 0 0</xyzOffset>
      <rpyOffset>0 0 0</rpyOffset>
    </plugin>
  </gazebo>

**/

std::string parent_frame_id = "map";

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
  static tf2_ros::TransformBroadcaster odom_br;
  geometry_msgs::TransformStamped transformStamped;

  // Publish TF from the given message
  transformStamped.header = msg->header;

  // check is msg frame_id and parent_frame_id are the same
  // overwrite it if not the same
  if (msg->header.frame_id != parent_frame_id) {
    transformStamped.header.frame_id = parent_frame_id;
    ROS_WARN_STREAM_ONCE("[p3d_link_tf] link " << parent_frame_id << " to " << msg->child_frame_id);
  } else {
    ROS_WARN_STREAM_ONCE("[p3d_link_tf] parent_frame_id and msg->header.frame_id are the same");
  }

  transformStamped.child_frame_id = msg->child_frame_id;
  transformStamped.transform.translation.x = msg->pose.pose.position.x;
  transformStamped.transform.translation.y = msg->pose.pose.position.y;
  transformStamped.transform.translation.z = msg->pose.pose.position.z;
  transformStamped.transform.rotation.x = msg->pose.pose.orientation.x;
  transformStamped.transform.rotation.y = msg->pose.pose.orientation.y;
  transformStamped.transform.rotation.z = msg->pose.pose.orientation.z;
  transformStamped.transform.rotation.w = msg->pose.pose.orientation.w;

  odom_br.sendTransform(transformStamped);
  ROS_WARN_ONCE("[p3d_link_tf] publish tf from [%s] to [%s]", transformStamped.header.frame_id.c_str(),
                transformStamped.child_frame_id.c_str());
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "p3d_link_tf");
  ros::NodeHandle nh("~");

  // ------------------------ read parameters --------------------------------
  if (!nh.getParam("parent_frame_id", parent_frame_id)) {
    ROS_WARN("[p3d_link_tf] Could not find [parent_frame_id] parameter");
  } else {
    ROS_INFO_STREAM("[p3d_link_tf], set parent_frame_id: " << parent_frame_id);
  }

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  ros::Subscriber sub = nh.subscribe("input", 10, odomCallback, ros::TransportHints().tcpNoDelay());

  ros::Rate loop_rate(50);

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
};
