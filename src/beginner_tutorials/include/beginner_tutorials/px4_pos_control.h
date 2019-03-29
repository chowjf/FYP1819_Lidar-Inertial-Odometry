#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include<mavros_msgs/ExtendedState.h>
#include <math.h>

using namespace geometry_msgs;
using namespace std;
using namespace ros;



// Subscribers
ros::Subscriber state_sub;
ros::Subscriber ext_state_sub;
ros::Subscriber pos_sub;

ros::ServiceClient arming_client;
ros::ServiceClient set_mode_client;

// Publishers
ros::Publisher local_pos_pub;

double t, t_last, t_traj;
