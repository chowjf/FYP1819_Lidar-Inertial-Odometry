// Programmed by: JF
// Date: 18 Mar 19
// This node broadcasts coordinate frames, /mavros/vision_pose/Pose to tf

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

void callback_vision_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z) );
  transform.setRotation( tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w) );
  br.sendTransform( tf::StampedTransform(transform, ros::Time::now(), "map", "vision_link") );
  // ROS_INFO("Sending...\n");
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster_node");

  int loopRate;
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  pn.param<int>("loop_rate", loopRate, 10);
  ros::Subscriber vision_pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", loopRate, &callback_vision_pose);
  ROS_INFO("Receiving data from /mavros/vision_pose\n");
  ros::spin();
  return 0;
}
