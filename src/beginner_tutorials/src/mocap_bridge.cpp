#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <tf/transform_datatypes.h>
#include <math.h>

double sample_time = 0.005;

geometry_msgs::PoseStamped mocap_pos_in, mocap_pos_out;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    mocap_pos_in = *msg;
}

geometry_msgs::PoseStamped local_pos;
void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    local_pos = *msg;
}

geometry_msgs::TwistStamped local_vel, mocap_vel;
void local_vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    local_vel = *msg;
}

int main(int argc, char ** argv)
{
    ros::init(argc,argv,"mocap_bridge_node");
    ros::NodeHandle nh;

    ros::Subscriber pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("Robot_1/pose", 10, pos_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, local_pos_cb);
    ros::Subscriber local_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 10, local_vel_cb);

    ros::Publisher pos_pub_mocap_ENU = nh.advertise<geometry_msgs::PoseStamped>("Robot_1/pose_ENU", 10);
    ros::Publisher pos_pub_mocap = nh.advertise<geometry_msgs::PoseStamped>("mavros/mocap/pose", 10);
//    ros::Publisher pos_pub_mocap = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);
    ros::Publisher vel_pub_mocap = nh.advertise<geometry_msgs::TwistStamped>("mavros/mocap/velocity", 1);

    ros::Rate rate(1/sample_time);

    double roll_out, pitch_out, yaw_out, pi = 22/7;
    double roll_in, pitch_in, yaw_in;
//    double roll_current, pitch_current, yaw_current;
    double roll_local, pitch_local, yaw_local;

    double mocap_pos_x_last = 0.0;
    double mocap_pos_y_last = 0.0;
    double mocap_pos_z_last = 0.0;

    std::vector<double> mocap_vel_x_unfiltered(20,0.0);
    std::vector<double> mocap_vel_y_unfiltered(20,0.0);
    std::vector<double> mocap_vel_z_unfiltered(20,0.0);

    double mocap_vel_x = 0.0;
    double mocap_vel_y = 0.0;
    double mocap_vel_z = 0.0;

    double det_t = sample_time;

    while(ros::ok())
    {

 // ** Rotation from NWU to ENU frame (Rotation ov 90 deg. about z axis)

        tf::Quaternion mocap_q_in(mocap_pos_in.pose.orientation.x, mocap_pos_in.pose.orientation.y, mocap_pos_in.pose.orientation.z, mocap_pos_in.pose.orientation.w);
        tf::Matrix3x3 mocap_m_in(mocap_q_in);
        mocap_m_in.getRPY(roll_in, pitch_in, yaw_in);

/*
        mocap_pos_out.pose.position.x = -mocap_pos_in.pose.position.y;
        mocap_pos_out.pose.position.y = mocap_pos_in.pose.position.x;
        mocap_pos_out.pose.position.z = mocap_pos_in.pose.position.z;

        mocap_pos_out.pose.orientation.x = mocap_pos_in.pose.orientation.x;
        mocap_pos_out.pose.orientation.y = mocap_pos_in.pose.orientation.y;
        mocap_pos_out.pose.orientation.z = sqrt(0.5) * mocap_pos_in.pose.orientation.z;
        mocap_pos_out.pose.orientation.w = sqrt(0.5) *mocap_pos_in.pose.orientation.w;

        mocap_pos_out.pose.position.x = -mocap_pos_in.pose.position.y;
        mocap_pos_out.pose.position.y = mocap_pos_in.pose.position.x;
        mocap_pos_out.pose.position.z = mocap_pos_in.pose.position.z;
*/
        mocap_pos_out.pose.position.x = mocap_pos_in.pose.position.x;
        mocap_pos_out.pose.position.y = mocap_pos_in.pose.position.y;
        mocap_pos_out.pose.position.z = mocap_pos_in.pose.position.z;

//        mocap_pos_out.pose.position.z = mocap_pos_in.pose.position.z - 0.04;  // distance between pixhawk and marker = 4 cm

        tf::Quaternion mocap_q_out(mocap_pos_out.pose.orientation.x, mocap_pos_out.pose.orientation.y, mocap_pos_out.pose.orientation.z, mocap_pos_out.pose.orientation.w);
        tf::Matrix3x3 mocap_m_out(mocap_q_out);
        mocap_m_out.getRPY(roll_out, pitch_out, yaw_out);
/*
        roll_out = -pitch_in;
        pitch_out = roll_in;
        yaw_out = yaw_in;
*/
        roll_out = roll_in;
        pitch_out = pitch_in;
        yaw_out = yaw_in;

        mocap_q_out.setRPY(roll_out,pitch_out,yaw_out);

//        mocap_pos_out.header.seq = local_pos.header.seq;
        mocap_pos_out.header.stamp = ros::Time::now();
        mocap_pos_out.header.frame_id = "fcu";

        mocap_pos_out.pose.orientation.x = mocap_q_out.getX();
        mocap_pos_out.pose.orientation.y = mocap_q_out.getY();
        mocap_pos_out.pose.orientation.z = mocap_q_out.getZ();
        mocap_pos_out.pose.orientation.w = mocap_q_out.getW();

//        mocap_pos_out = mocap_pos_in;

        pos_pub_mocap.publish(mocap_pos_out);

        pos_pub_mocap_ENU.publish(mocap_pos_out);


        tf::Quaternion q_local(local_pos.pose.orientation.x, local_pos.pose.orientation.y, local_pos.pose.orientation.z, local_pos.pose.orientation.w);
        tf::Matrix3x3 m_local(q_local);
        m_local.getRPY(roll_local, pitch_local, yaw_local);


//         ROS_INFO("mocap X = %f m",mocap_pos_out.pose.position.x);
//         ROS_INFO("mocap Y = %f m",mocap_pos_out.pose.position.y);
//         ROS_INFO("mocap Z = %f m",mocap_pos_out.pose.position.z);
//         ROS_INFO("----------------------------------------");
//         ROS_INFO("Local X = %f m",local_pos.pose.position.x);
//         ROS_INFO("Local Y = %f m",local_pos.pose.position.y);
//         ROS_INFO("Local Z = %f m",local_pos.pose.position.z);
//         ROS_INFO("----------------------------------------");

        std::cout<<"mocap X = "<<mocap_pos_out.pose.position.x<<" m \n";
        std::cout<<"mocap Y = "<<mocap_pos_out.pose.position.y<<" m \n";
        std::cout<<"mocap Z = "<<mocap_pos_out.pose.position.z<<" m \n";
        std::cout<<"---------------------------------------- \n";
        std::cout<<"Local X = "<<local_pos.pose.position.x<<" m \n";
        std::cout<<"Local Y = "<<local_pos.pose.position.y<<" m \n";
        std::cout<<"Local Z = "<<local_pos.pose.position.z<<" m \n";
        std::cout<<"---------------------------------------- \n";
        std::cout<<"---------------------------------------- \n";

//         ROS_INFO("mocap Roll = %f deg",roll_out*(180/pi));
//         ROS_INFO("mocap Pitch = %f deg",pitch_out*(180/pi));
//         ROS_INFO("mocap Yaw = %f deg",yaw_out*(180/pi));
//         ROS_INFO("----------------------------------------");
//         ROS_INFO("Local Roll = %f deg",roll_local*(180/pi));
//         ROS_INFO("Local Pitch = %f deg",pitch_local*(180/pi));
//         ROS_INFO("Local Yaw = %f deg",yaw_local*(180/pi));
//         ROS_INFO("----------------------------------------");
//         ROS_INFO("----------------------------------------");

        std::cout<<"mocap Roll = "<<roll_out*(180/pi)<<" deg \n";
        std::cout<<"mocap Pitch = "<<pitch_out*(180/pi)<<" deg \n";
        std::cout<<"mocap Yaw = "<<yaw_out*(180/pi)<<" deg \n";
        std::cout<<"---------------------------------------- \n";
        std::cout<<"Local Roll = "<<roll_local*(180/pi)<<" deg \n";
        std::cout<<"Local Pitch = "<<pitch_local*(180/pi)<<" deg \n";
        std::cout<<"Local Yaw = "<<yaw_local*(180/pi)<<" deg \n";
        std::cout<<"---------------------------------------- \n";
        std::cout<<"---------------------------------------- \n";

        mocap_vel.header.stamp = ros::Time::now();

        int i = 0;
        while (i < mocap_vel_x_unfiltered.size()-1)
        {
            mocap_vel_x_unfiltered[i] = mocap_vel_x_unfiltered[i+1];
            mocap_vel_y_unfiltered[i] = mocap_vel_y_unfiltered[i+1];
            mocap_vel_z_unfiltered[i] = mocap_vel_z_unfiltered[i+1];
            ++i;
        }
        mocap_vel_x_unfiltered[i] = (mocap_pos_out.pose.position.x - mocap_pos_x_last)/det_t;
        mocap_vel_y_unfiltered[i] = (mocap_pos_out.pose.position.y - mocap_pos_y_last)/det_t;
        mocap_vel_z_unfiltered[i] = (mocap_pos_out.pose.position.z - mocap_pos_z_last)/det_t;
        mocap_vel_x = 0.0;
        mocap_vel_y = 0.0;
        mocap_vel_z = 0.0;
        for (int i=0; i<mocap_vel_x_unfiltered.size(); ++i)
        {
            mocap_vel_x += mocap_vel_x_unfiltered[i];
            mocap_vel_y += mocap_vel_y_unfiltered[i];
            mocap_vel_z += mocap_vel_z_unfiltered[i];
        }
        mocap_vel_x = mocap_vel_x/mocap_vel_x_unfiltered.size();
        mocap_vel_y = mocap_vel_y/mocap_vel_y_unfiltered.size();
        mocap_vel_z = mocap_vel_z/mocap_vel_z_unfiltered.size();
        mocap_vel.twist.linear.x = mocap_vel_x;
        mocap_vel.twist.linear.y = mocap_vel_y;
        mocap_vel.twist.linear.z = mocap_vel_z;
/*
        mocap_vel.twist.linear.x = local_vel.twist.linear.x;
        mocap_vel.twist.linear.y = local_vel.twist.linear.y;
        mocap_vel.twist.linear.z = local_vel.twist.linear.z;
*/
        mocap_vel.twist.angular.x = local_vel.twist.angular.x;
        mocap_vel.twist.angular.y = local_vel.twist.angular.y;
        mocap_vel.twist.angular.z = local_vel.twist.angular.z;

        vel_pub_mocap.publish(mocap_vel);

        mocap_pos_x_last = mocap_pos_out.pose.position.x;
        mocap_pos_y_last = mocap_pos_out.pose.position.y;
        mocap_pos_z_last = mocap_pos_out.pose.position.z;

        ros::spinOnce();
        rate.sleep();
    }

	return 0;
}
