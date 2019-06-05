
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
 //#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>

#include <mavros_msgs/CommandTOL.h>

#include <mavros_msgs/SetMode.h>

#include <mavros_msgs/State.h>
 //#include "sensor_msgs/Range.h"
#include <tf/transform_datatypes.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
// custom messages
#include <local_mapping/detected_object.h>
#include <srf08_ranging/obstacle_distance.h>

#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <cstddef>
#include <math.h>
#include <vector>
#include <algorithm>

//
int u;
int v;
//
int landing = 0;
int count = 0;
float takeoff_alt = 1.5;
float local_x;
float local_y;
float local_z;
float yaw_ornt;
float yawt;       

int obstacle_up =200.0; int obstacle_right =200.0; int obstacle_left =200.0;
float obstacle_front = 200.0; float obstacle_down = 200.0;

mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose_sp;
ros::Publisher pose_sp_pub;

enum mission_type
{
mission_takeoff = 0,
mission_wall,
mission_window,
mission_pole,
mission_pipe,
mission_tree,
mission_net ,
mission_landing	
};

enum track_window_cmd
{
 search,
 align,
 approach,
};

enum window_tracking_state
{
 searching,
 aligning,
 approaching,
 passed
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


struct windows_location
{
  float win_Xc;
  float win_Yc;
  float win_Zc;
  
};

struct windows
{
	std::vector<windows_location> wndw;
}wins;

float Nwpx, Nwpy, Nwpz; 
float WXm, WYm, WZm, Xtot, Ytot, Ztot;
float Xvar_max = 0.0; 
float Yvar_max = 0.0;
float Zvar_max = 0.0;

float Xcam, Ycam, Zcam;
std::string target_name = "window";
void wait_cycle(int cycle, ros::Rate r );


void limitWin_location(float& x, float& y, float& z);
void winlocation_stat(windows wds,float& Wxm, float& Wym, float& Wzm, float& Xt,float& Yt, float& Zt, float& Xvar, float& Yvar,float& Zvar );
void publish_pos_sp(ros::Rate r);
void window_scan (int& p, int& d ,float& x, float& y, float& z,float yaw,ros::Rate r,track_window_cmd& window_cmd);
void pause4search(int cycle, ros::Rate r, track_window_cmd& win_cmd);
void find_nextsafe_wp (float& wpx, float& wpy, float& wpz , mission_type mission, track_window_cmd& win_cmd, int& d); 