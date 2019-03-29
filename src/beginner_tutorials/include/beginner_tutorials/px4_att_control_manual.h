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

// Gains
double k_p_roll, k_i_roll, k_d_roll;
double k_p_yaw, k_i_yaw, k_d_yaw;

// Scales
double min_scale_roll, max_scale_roll;
double min_scale_yaw, max_scale_yaw;

double t, t_last, t_traj;
double roll_des, pitch_des, yaw_des, throt_des, pi = 22/7;
double roll_current, pitch_current, yaw_current, throt_current;

double cmd_scaled_roll, cmd_scaled_yaw;

class PID
{
    public:
        PID( double dt, double scale_min, double scale_max, double k_p, double k_i, double k_d);
        ~PID();
        double calculate( double desired, double current);

    private:
        double _dt;
        double _scale_min;
        double _scale_max;
        double _k_p;
        double _k_i;
        double _k_d;
        double _pre_error;
        double _integral;
};


