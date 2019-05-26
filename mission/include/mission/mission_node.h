
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
 //#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>

#include <mavros_msgs/CommandTOL.h>

#include <mavros_msgs/SetMode.h>

#include <mavros_msgs/State.h>
 //#include "sensor_msgs/Range.h"
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
int landing = 0;
int count = 0;
float takeoff_alt = 1.5;
float local_x;
float local_y;
float local_z;
float yaw_ornt;
float yawt;       

int obstacle_up, obstacle_right, obstacle_left;
float obstacle_front;

mavros_msgs::State current_state;

enum mission_type
{
mission_takeoff = 0,
mission_window,
mission_pole,
mission_pipe,
mission_tree,
mission_net ,
mission_landing	
};

enum mission_state
{
success,
fail
};