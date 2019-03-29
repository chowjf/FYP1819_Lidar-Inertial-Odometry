#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <math.h>

#include <tf/transform_datatypes.h>

using namespace geometry_msgs;
using namespace std;
using namespace ros;



// Subscribers
ros::Subscriber state_sub;
ros::Subscriber att_sub;


// Publishers
ros::Publisher local_att_pub;
ros::Publisher local_thro_pub;
ros::Publisher current_roll_pub;
ros::Publisher current_yaw_pub;
ros::Publisher command_roll_pub;
ros::Publisher command_yaw_pub;

double t, t_last, t_traj;
double roll, pitch, yaw, pi = 22/7;
double roll_ref, pitch_ref, yaw_ref, roll_des, pitch_des, yaw_des;
double roll_current, pitch_current, yaw_current;
