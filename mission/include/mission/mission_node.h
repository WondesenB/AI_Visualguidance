
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

#include <local_mapping/detected_object.h>


#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <cstddef>
#include <math.h>
#include <vector>
#include <algorithm>


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

enum track_window_cmd
{
 window_search,
 align_vertical,
 align_sideway,
 approach_forward,
};

enum window_tracking_state
{
 window_searching,
 aligning_vertical,
 aligning_sideway,
 approaching_forward,
 window_passed
};

enum mission_state
{
success,
fail
};

struct detected_object_detail
{
std::string obj_name;
float  Ymin;
float  Zmin;
float  X;
float  Y;
float  Z;
float  distance;
float  area;	
};

struct detected_objects
{
 std::vector<detected_object_detail> object;
}dobj;

void wait_cycle(int cycle, ros::Rate r );

void limitWin_location(float& x, float& y, float& z);