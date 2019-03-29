/*
Copyright Marcel Stüttgen 2015 <stuettgen@fh-aachen.de>
simple ros node that will publish ActuatorControl message to /mavros/actuator_controls
*/

#include <beginner_tutorials/px4_pos_control.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pos, pos_ref_start, pos_ref;
void pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    current_pos = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_pos_control");
    ros::NodeHandle nh;
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, pos_cb);

    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
//    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
//    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Rate rate(20.0);

    while(ros::ok() && current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    pos_ref_start = current_pos;

    geometry_msgs::PoseStamped pos_cmd, last_pos_cmd;
    pos_cmd.pose.position.x = pos_ref_start.pose.position.x;
    pos_cmd.pose.position.y = pos_ref_start.pose.position.y;
    pos_cmd.pose.position.z = pos_ref_start.pose.position.z;

    last_pos_cmd = pos_cmd;

//    mavros_msgs::SetMode offb_set_model;
//    offb_set_model.request.custom_mode = "OFFBOARD";

//    mavros_msgs::CommandBool arm_cmd;
//    arm_cmd.request.value = true;

    double traj_start = 0;

    while(ros::ok())
    {
        if( current_state.mode != "OFFBOARD")
        {
            ROS_INFO("OFBOARD mode is not enabled!");
        }
        if( !current_state.armed)
        {
            ROS_INFO("Vehicle is not armed!");
        }

        if( current_state.mode == "ALTCTL")
        {
            pos_ref = current_pos;
            traj_start = 0;
        }

        else
        {
            pos_cmd.pose.position.x = pos_ref.pose.position.x;
            pos_cmd.pose.position.y = pos_ref.pose.position.y + 1;
            pos_cmd.pose.position.z = pos_ref.pose.position.z;
            pos_cmd.pose.orientation.z = pos_ref.pose.orientation.z;
            pos_cmd.pose.orientation.w = pos_ref.pose.orientation.w;

            if (abs(current_pos.pose.position.y - (pos_ref.pose.position.y + 1)) <= 0.05)
            {
                traj_start = 1;
                t = ros::Time::now().toSec();
                t_last = ros::Time::now().toSec();
                while(ros::ok() && t <= t_last + 5)
                {
                    ROS_INFO("Wait for 5 secs!");
                    local_pos_pub.publish(pos_cmd);
                    last_pos_cmd = pos_cmd;
                    t = ros::Time::now().toSec();

                    ros::spinOnce();
                    rate.sleep();
                }
                t_last = t;
            }


            while(ros::ok() && (current_state.mode == "OFFBOARD") && (traj_start == 1))
            {
                t = ros::Time::now().toSec();
                t_traj = t - t_last;

                pos_cmd.pose.position.x = pos_ref.pose.position.x + sin(t_traj/2);
                pos_cmd.pose.position.y = pos_ref.pose.position.y + cos(t_traj/2);
                pos_cmd.pose.position.z = pos_ref.pose.position.z;

                local_pos_pub.publish(pos_cmd);
                last_pos_cmd = pos_cmd;

                ROS_INFO("Circular Trajectory following!");
                ROS_INFO("Reference Time t_traj = %f",t_traj);
                ROS_INFO("Reference center x = %f",pos_ref.pose.position.x);
                ROS_INFO("Reference center y = %f",pos_ref.pose.position.y);
                ROS_INFO("Reference center z = %f",pos_ref.pose.position.z);

                ros::spinOnce();
                rate.sleep();

            }

            local_pos_pub.publish(pos_cmd);
            last_pos_cmd = pos_cmd;
        }


        /*if(current_pos.pose.position.x == last_pos_cmd.pose.position.x &&
           current_pos.pose.position.y == last_pos_cmd.pose.position.y &&
           current_pos.pose.position.z == last_pos_cmd.pose.position.z)
        {*/

        /*}
        else
        {
            local_pos_pub.publish(last_pos_cmd);
        }*/


        ROS_INFO("Reference Position x = %f",pos_ref.pose.position.x);
        ROS_INFO("Reference Position y = %f",pos_ref.pose.position.y);
        ROS_INFO("Reference Position z = %f",pos_ref.pose.position.z);
        ROS_INFO("Reference Orientation z = %f",pos_ref.pose.orientation.z);
        ROS_INFO("Reference Orientation w = %f",pos_ref.pose.orientation.w);

        local_pos_pub.publish(pos_cmd);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
