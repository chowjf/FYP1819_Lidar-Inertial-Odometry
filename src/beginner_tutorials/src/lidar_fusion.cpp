// Programmed by: JF
// Date： 15 Mar 19
// This node is used to subscribe /Pose msgs from hector slam and /Range msgs from
// TFmini, fusing them become complete 3D Pose msgs publishing to /vision_pose topic
//----updated 29 Mar:
//include 0.08m offset to CG of base_link

#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Range.h>
#include <tf/transform_datatypes.h>
#include <math.h>

//sampling rate in Hz
int loopRate = 10; double offset = 0.08;

//Variables
geometry_msgs::PoseStamped vision_pose, hector_pose, local_pose;
sensor_msgs::Range tfmini_alti;

void callback_hector_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //ROS_INFO("Got data from hector_pose: %f, %f, %f \n", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    hector_pose = *msg;
}

void callback_tfmini_alti(const sensor_msgs::Range::ConstPtr& msg)
{
    //ROS_INFO("Got data from tfmini_alti: %f \n", msg->range);
    tfmini_alti = *msg;
}

void callback_local_pose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //ROS_INFO("Got data from local_pos\n");
    local_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_fusion_node");

    std::string p_base_frame_;
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    pn.param<std::string>("base_frame", p_base_frame_, "base_fusion");
    // pn.param("lidar_topic", p_lidar_topic_, std::string("/mavros/distance_sensor/tfmini_pub"));
    pn.param<int>("loop_rate", loopRate, 10);
    pn.param<int>("offset_altitude", offset, 0.08);

    ros::Subscriber hector_pose_sub = n.subscribe<geometry_msgs::PoseStamped>("/slam_out_pose",loopRate, callback_hector_pose);
    ros::Subscriber tfmini_alti_sub = n.subscribe<sensor_msgs::Range>("/mavros/distance_sensor/tfmini_pub", loopRate, callback_tfmini_alti);
    ros::Subscriber local_pos_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", loopRate, callback_local_pose);

    ros::Publisher vision_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",loopRate);

    ros::Rate loop_rate(loopRate);

    double local_roll, local_pitch, hector_yaw, dummy; //yaw we fuse directly from hector_slam
    ROS_INFO("Waiting for input...\n");

    while (ros::ok())
    {
        loop_rate.sleep();
        //Rotation from hector_slam from Quaternion to getRPY
        tf::Quaternion local_q(local_pose.pose.orientation.x, local_pose.pose.orientation.y, local_pose.pose.orientation.z, local_pose.pose.orientation.w);
        tf::Matrix3x3 local_m(local_q);
        local_m.getRPY(local_roll, local_pitch, dummy);

        tf::Quaternion hector_q(hector_pose.pose.orientation.x, hector_pose.pose.orientation.y, hector_pose.pose.orientation.z, hector_pose.pose.orientation.w);
        tf::Matrix3x3 hector_m(hector_q);
        hector_m.getRPY(dummy, dummy, hector_yaw);

        tf::Quaternion vision_q;
        vision_q.setRPY(local_roll, local_pitch, hector_yaw);

        vision_pose.header.stamp = ros::Time::now();
        vision_pose.header.frame_id = p_base_frame_; //optional. Works fine without frame_id
        vision_pose.pose.position.x = hector_pose.pose.position.x;
        vision_pose.pose.position.y = hector_pose.pose.position.y;
        vision_pose.pose.position.z = tfmini_alti.range + offset;
        vision_pose.pose.orientation.x = vision_q.getX();
        vision_pose.pose.orientation.y = vision_q.getY();
        vision_pose.pose.orientation.z = vision_q.getZ();
        vision_pose.pose.orientation.w = vision_q.getW();
        vision_pub.publish(vision_pose);
        // ROS_INFO("%d", loopRate);
        ros::spinOnce();
    }
    return 0;
}
