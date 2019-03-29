#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCOut.h>
#include<std_msgs/UInt16MultiArray.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

mavros_msgs::RCOut current_rcout;
void rcout_cb(const mavros_msgs::RCOut::ConstPtr& msg)
{
    current_rcout = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state");
    ros::NodeHandle nh;

    // Subscribers
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 50, state_cb);
    ros::Subscriber rcout_sub = nh.subscribe<mavros_msgs::RCOut>("mavros/rc/out", 50, rcout_cb);

    // Publishers
    ros::Publisher state_pub = nh.advertise<std_msgs::Float64>("state/current_state", 10);
    ros::Publisher rcout_pub = nh.advertise<std_msgs::UInt16MultiArray>("rc/current_rcout", 10);

    ros::Rate rate(100.0);

    int print_flag = -1;
    std_msgs::Float64 current_state_value;
    std_msgs::UInt16MultiArray current_rcout_value;

    while(ros::ok())
    {
        current_rcout_value.data = current_rcout.channels;
        ROS_INFO_ONCE("/mavros/state -> /state/current_state");
        ROS_INFO_ONCE("/mavros/rc/out -> /rc/current_rcout");

        if( !current_state.armed)
            ROS_INFO_ONCE("Vehicle is not armed!");
        else
            ROS_INFO_ONCE("Vehicle is armed!");

        if(current_state.mode == "MANUAL")
        {
            current_state_value.data = 0;
            if (print_flag != current_state_value.data)
            {
                ROS_INFO("********************************");
                ROS_INFO("MANUAL flight mode!");
                ROS_INFO("current_state_value = %f",current_state_value.data);
                print_flag = current_state_value.data;
            }
        }
        if(current_state.mode == "STABILIZED")
        {
            current_state_value.data = 1;
            if (print_flag != current_state_value.data)
            {
                ROS_INFO("********************************");
                ROS_INFO("STABILIZED flight mode!!");
                ROS_INFO("current_state_value = %f",current_state_value.data);
                print_flag = current_state_value.data;
            }
        }
        if(current_state.mode == "ALTCTL")
        {
            current_state_value.data = 2;
            if (print_flag != current_state_value.data)
            {
                ROS_INFO("********************************");
                ROS_INFO("ALTCTL flight mode!!");
                ROS_INFO("current_state_value = %f",current_state_value.data);
                print_flag = current_state_value.data;
            }
        }
        if(current_state.mode == "POSCTL")
        {
            current_state_value.data = 3;
            if (print_flag != current_state_value.data)
            {
                ROS_INFO("********************************");
                ROS_INFO("POSCTL flight mode!!");
                ROS_INFO("current_state_value = %f",current_state_value.data);
                print_flag = current_state_value.data;
            }
        }
        if(current_state.mode == "OFFBOARD")
        {
            current_state_value.data = 4;
            if (print_flag != current_state_value.data)
            {
                ROS_INFO("********************************");
                ROS_INFO("Vehicle is OFFBOARD!");
                ROS_INFO("current_state_value = %f",current_state_value.data);
                print_flag = current_state_value.data;
            }
        }

        state_pub.publish(current_state_value);
        rcout_pub.publish(current_rcout_value);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
