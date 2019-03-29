/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>
#include <sensor_msgs/TimeReference.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/GlobalPositionTarget.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}
sensor_msgs::TimeReference current_time;
int time_rf(const sensor_msgs::TimeReference::ConstPtr& msg){
    current_time = *msg;
}
geometry_msgs::PoseStamped current_pos;
double local_pos(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pos = *msg;
}
geometry_msgs::PoseStamped mocap_pos;
double mocap_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    mocap_pos = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber time_ref_sub = nh.subscribe<sensor_msgs::TimeReference>
            ("mavros/time_reference", 10, time_rf);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 100, local_pos);
    ros::Subscriber mocap_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/mocap/pose", 10, mocap_pos_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 100);
    ros::Publisher act_ctrl_pub = nh.advertise<mavros_msgs::ActuatorControl>
            ("mavros/actuator_control", 100);
    ros::Publisher local_att_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_attitude/attitude", 100);
    ros::Publisher local_rawatt_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", 100);
    ros::Publisher rc_io_pub = nh.advertise<mavros_msgs::OverrideRCIn>
            ("mavros/rc/override", 100);
    ros::Publisher set_acc_pub = nh.advertise<geometry_msgs::Vector3Stamped>
            ("mavros/setpoint_accel/accel", 100);
    ros::Publisher ang_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_attitude/cmd_vel", 100);
    ros::Publisher att_throt_pub = nh.advertise<std_msgs::Float64>
            ("mavros/setpoint_attitude/att_throttle", 100);
    ros::Publisher lin_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("mavros/setpoint_velocity/cmd_vel", 100);
    ros::Publisher loc_target_pub = nh.advertise<mavros_msgs::PositionTarget>
            ("mavros/setpoint_raw/local", 100);
    ros::Publisher glo_target_pub = nh.advertise<mavros_msgs::GlobalPositionTarget>
            ("mavros/setpoint_raw/global", 100);
    ros::Publisher att_target_pub = nh.advertise<mavros_msgs::AttitudeTarget>
            ("mavros/setpoint_raw/attitude", 100);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    double t = 0;
    double t_old;
    double roll; double pitch; double yaw; double throttle;
    double ysqr, croll, cpitch, cyaw;
    double droll, dpitch, dyaw;

	  double t0 = std::cos(0 * 0.5);  // cos(yaw*0.5) (in radians)
	  double t1 = std::sin(0 * 0.5);  // sin(yaw*0.5)
	  double t2 = std::cos(0 * 0.5);  // cos(roll*0.5)
	  double t3 = std::sin(0 * 0.5);  // sin(roll*0.5)
	  double t4 = std::cos(0 * 0.5);  // cos(pitch*0.5)
	  double t5 = std::sin(0 * 0.5);  // sin(pitch*0.5)

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1;

    // pose.pose.orientation.x = t0 * t3 * t4 - t1 * t2 * t5;
    // pose.pose.orientation.y = t0 * t2 * t5 + t1 * t3 * t4;
    // pose.pose.orientation.z = t1 * t2 * t4 - t0 * t3 * t5;
    // pose.pose.orientation.w = t0 * t2 * t4 + t1 * t3 * t5;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";
    //
    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    t_old = ros::Time::now().toSec();

    int count = 1; double ii=0; double st = 0;
    double error_z, error_z_old, error_z_dot;
    t=0;

    while(ros::ok()){
        // if( current_state.mode != "OFFBOARD" &&
        //     (ros::Time::now() - last_request > ros::Duration(5.0))){
        //     if( set_mode_client.call(offb_set_mode) &&
        //         offb_set_mode.response.success){
        //         ROS_INFO("Offboard enabled");
        //     }
        //     last_request = ros::Time::now();
        // } else {
        //     if( !current_state.armed &&
        //         (ros::Time::now() - last_request > ros::Duration(5.0))){
        //         if( arming_client.call(arm_cmd) &&
        //             arm_cmd.response.success){
        //             ROS_INFO("Vehicle armed");
        //         }
        //         last_request = ros::Time::now();
        //     }
        // }

        // ROS_INFO("%.*f", 2, t);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = ros::Time::now();
        pose.header.seq = count;
        pose.header.frame_id = "";

        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 1;

        // pose.pose.orientation.x = t0 * t3 * t4 - t1 * t2 * t5;
        // pose.pose.orientation.y = t0 * t2 * t5 + t1 * t3 * t4;
        // pose.pose.orientation.z = t1 * t2 * t4 - t0 * t3 * t5;
        // pose.pose.orientation.w = t0 * t2 * t4 + t1 * t3 * t5;


        roll = 0;
        pitch = 0;
        yaw = 0;
        throttle = 0;

        mavros_msgs::ActuatorControl actuators;
        actuators.header.stamp = ros::Time::now();
        actuators.header.seq = count;
        actuators.header.frame_id = "";

        actuators.group_mix = 1;
        actuators.controls = {roll, pitch, yaw, throttle, 0, 0, 0, 0};


        mavros_msgs::OverrideRCIn rc_over;
        rc_over.channels = {0, 0, 0, 0, 0, 0, 0, 0};


        geometry_msgs::Vector3Stamped accelerations;
        accelerations.header.stamp = ros::Time::now();
        accelerations.header.seq = count;
        accelerations.header.frame_id = "";

        accelerations.vector.x = 0;
        accelerations.vector.y = 0;
        accelerations.vector.z = 0;


        geometry_msgs::TwistStamped velocities;
        velocities.header.stamp = ros::Time::now();
        velocities.header.seq = count;
        velocities.header.frame_id = "";

        velocities.twist.linear.x = 0.5;
        velocities.twist.linear.y = 0;
        velocities.twist.linear.z = 0;

        velocities.twist.angular.x = 0;
        velocities.twist.angular.y = 0;
        velocities.twist.angular.z = 0;


        std_msgs::Float64 throt;
        throt.data = 0.6;


        mavros_msgs::PositionTarget raw_target_loc;
        raw_target_loc.header.stamp = ros::Time::now();
        raw_target_loc.header.seq = count;
        raw_target_loc.header.frame_id = "";
        raw_target_loc.coordinate_frame = 1;
        raw_target_loc.type_mask = 1 |                           //x.pos
                               2 |                           //y.pos
                               4 |                           //z.pos
                               //8 |                           //x.vel
                               //16 |                          //y.vel
                               //32 |                          //z.vel
                               64 |                          //x.acc
                               128 |                         //y.acc
                               256 |                         //z.acc
                               512 |                         //force
                               //1024 |                        //yaw
                               2048                          //yaw_rate
                               ;

        raw_target_loc.position.x = 0;
        raw_target_loc.position.y = 0;
        raw_target_loc.position.z = 0;

        raw_target_loc.velocity.x = 0;
        raw_target_loc.velocity.y = 0;
        raw_target_loc.velocity.z = 0;

        raw_target_loc.acceleration_or_force.x = 0;
        raw_target_loc.acceleration_or_force.y = 0;
        raw_target_loc.acceleration_or_force.z = 0;

        raw_target_loc.yaw = 1.5708;
        raw_target_loc.yaw_rate = 0;


        mavros_msgs::GlobalPositionTarget raw_target_glo;
        raw_target_glo.header.stamp = ros::Time::now();
        raw_target_glo.header.seq = count;
        raw_target_glo.header.frame_id = "";
        raw_target_glo.coordinate_frame = 5;
        raw_target_glo.type_mask = 1 |                           //latt
                                   2 |                           //long
                                   4 |                           //alti
                                   8 |                           //x.vel
                                   16 |                          //y.vel
                                   32 |                          //z.vel
                                   64 |                          //x.acc
                                   128 |                         //y.acc
                                   256 |                         //z.acc
                                   512 |                         //force
                                   1024 |                        //yaw
                                   2048                          //yaw_rate
                                   ;

        raw_target_glo.latitude = 0;
        raw_target_glo.longitude = 0;
        raw_target_glo.altitude = 0;

        raw_target_glo.velocity.x = 0;
        raw_target_glo.velocity.y = 0;
        raw_target_glo.velocity.z = 0;

        raw_target_glo.acceleration_or_force.x = 0;
        raw_target_glo.acceleration_or_force.y = 0;
        raw_target_glo.acceleration_or_force.z = 0;

        raw_target_glo.yaw = 3.1416;
        raw_target_glo.yaw_rate = 0;


        mavros_msgs::AttitudeTarget raw_target_att;
        raw_target_att.header.stamp = ros::Time::now();
        raw_target_att.header.seq = count;
        raw_target_att.header.frame_id = "";
        raw_target_att.type_mask = 1 |                           //rollrate
                                   2 |                           //pitchrate
                                   4 //|                           //yawrate
                                   //64 |                          //thrust
                                   //128                           //att
                                   ;

        droll = 0.0;
        dpitch = 0.0;
        dyaw = 0.0;

        t0 = std::cos(dyaw * 0.5);  // cos(yaw*0.5) (in radians)
        t1 = std::sin(dyaw * 0.5);  // sin(yaw*0.5)
        t2 = std::cos(droll * 0.5);  // cos(roll*0.5)
        t3 = std::sin(droll * 0.5);  // sin(roll*0.5)
        t4 = std::cos(dpitch * 0.5);  // cos(pitch*0.5)
        t5 = std::sin(dpitch * 0.5);  // sin(pitch*0.5)

        raw_target_att.orientation.x = t0 * t3 * t4 - t1 * t2 * t5;
        raw_target_att.orientation.y = t0 * t2 * t5 + t1 * t3 * t4;
        raw_target_att.orientation.z = t1 * t2 * t4 - t0 * t3 * t5;
        raw_target_att.orientation.w = t0 * t2 * t4 + t1 * t3 * t5;

        raw_target_att.body_rate.x = 0;
        raw_target_att.body_rate.y = 0;
        raw_target_att.body_rate.z = 0;

        //raw_target_att.thrust = 0.62;

        // PD tuning
        error_z_old = error_z;
        error_z = 1-mocap_pos.pose.position.z;
        error_z_dot = (error_z-error_z_old)/0.02;
        // ROS_INFO("%.*f", 2, mocap_pos.pose.position.z);

        raw_target_att.thrust = 0.5+1.2*error_z + 0.4*error_z_dot;
        // raw_target_att.thrust = 0.5+1.5*error_z + 0.45*error_z_dot;

        // att_target_pub.publish(raw_target_att);

         local_pos_pub.publish(pose);



        // if(t<10)
        // {
        //   local_pos_pub.publish(pose);
        // }
        // if(t>=10 && t<=12)
        // {
        //   att_target_pub.publish(raw_target_att);
        // }
        // if(t>12)
        // {
        //   local_pos_pub.publish(pose);
        // }

//         if(st==1)
//         {
//           // lin_vel_pub.publish(velocities2);
//           loc_target_pub.publish(raw_target_loc);
//           // glo_target_pub.publish(raw_target_glo);
//         }
//         if(t<15)
//         {
//           local_pos_pub.publish(pose);
//         }
//         if(t>=15 && t<18)
//         {
//           lin_vel_pub.publish(velocities);
//         }
//         if(t>=18 && st!=1)
//         {
//           // att_target_pub.publish(raw_target_att);
//           local_att_pub.publish(pose);
//           //Quaternion to Euler
//           ysqr = current_pos.pose.orientation.y*current_pos.pose.orientation.y;
//           t0 = +2.0*(current_pos.pose.orientation.w*current_pos.pose.orientation.x+current_pos.pose.orientation.y*current_pos.pose.orientation.z);
//           t1 = +1.0-2.0*(current_pos.pose.orientation.x*current_pos.pose.orientation.x+ysqr);
//           croll = std::atan2(t0, t1);
//           t2 = +2.0*(current_pos.pose.orientation.w*current_pos.pose.orientation.y-current_pos.pose.orientation.z*current_pos.pose.orientation.x);
//           t2 = t2 > 1.0 ? 1.0 : t2;
//           t2 = t2 < -1.0 ? -1.0 : t2;
//           cpitch = std::asin(t2);
//           t3 = +2.0*(current_pos.pose.orientation.w*current_pos.pose.orientation.z+current_pos.pose.orientation.x*current_pos.pose.orientation.y);
//           t4 = +1.0-2.0*(ysqr+current_pos.pose.orientation.z*current_pos.pose.orientation.z);
//           cyaw = std::atan2(t3, t4);
//           if(cyaw>3.05)
//           {
//             st=1;
//           }
//         }
//
// //        local_pos_pub.publish(pose);
// //        local_att_pub.publish(pose);
// //        act_ctrl_pub.publish(actuators);
// //        rc_io_pub.publish(rc_over);
// //        set_acc_pub.publish(accelerations);
//       lin_vel_pub.publish(velocities);
// //        ang_vel_pub.publish(velocities);
// //        att_throt_pub.publish(throt);
// //        loc_target_pub.publish(raw_target_loc);
// //        glo_target_pub.publish(raw_target_glo);
// //        att_target_pub.publish(raw_target_att);
//
//
//         // ROS_INFO("Desired roll, pitch, yaw");
//         // ROS_INFO("%.*f", 2, droll);
//         // ROS_INFO("%.*f", 2, dpitch);
//         // ROS_INFO("%.*f", 2, dyaw);
//         ysqr = current_pos.pose.orientation.y*current_pos.pose.orientation.y;
//         t0 = +2.0*(current_pos.pose.orientation.w*current_pos.pose.orientation.x+current_pos.pose.orientation.y*current_pos.pose.orientation.z);
//         t1 = +1.0-2.0*(current_pos.pose.orientation.x*current_pos.pose.orientation.x+ysqr);
//         croll = std::atan2(t0, t1);
//         t2 = +2.0*(current_pos.pose.orientation.w*current_pos.pose.orientation.y-current_pos.pose.orientation.z*current_pos.pose.orientation.x);
//         t2 = t2 > 1.0 ? 1.0 : t2;
//         t2 = t2 < -1.0 ? -1.0 : t2;
//         cpitch = std::asin(t2);
//         t3 = +2.0*(current_pos.pose.orientation.w*current_pos.pose.orientation.z+current_pos.pose.orientation.x*current_pos.pose.orientation.y);
//         t4 = +1.0-2.0*(ysqr+current_pos.pose.orientation.z*current_pos.pose.orientation.z);
//         cyaw = std::atan2(t3, t4);
//         // ROS_INFO("Current roll, pitch, yaw");
//         // ROS_INFO("%.*f", 2, croll);
//         // ROS_INFO("%.*f", 2, cpitch);
//         // ROS_INFO("%.*f", 2, cyaw);
        ros::spinOnce();
        count++;
        rate.sleep();
//        t = ros::Time::now().toSec() - t_old;
        t = t + 0.02;
        // ROS_INFO("%.*f", 2, t);
    }

    return 0;
}
