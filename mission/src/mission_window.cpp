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
//---------------------
#include <object_localization/detected_object.h>
#include <srf08_ranging/obstacle_distance.h>
//
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
int    detection_count; 
};

struct detected_objects
{
 std::vector<detected_object_detail> object;
}dobj;

float WXc;
float WYc;
float WZc;

std::string target_name = "window";
int   detect_count =0;
//----------------------
int count=0;
float takeoff_alt = 1.3;
float local_x;
float local_y;
float local_z;
float yaw_ornt;
float yawt;       

mavros_msgs::State current_state;
geometry_msgs::PoseStamped pose;;
ros::Publisher pose_sp_pub;

void publish_pos_sp(ros::Rate r);

void detected_object_cb(const object_localization::detected_object::ConstPtr&  msg)
{
   
 
  dobj.object.clear();
  for (int i =0; i<msg->info.size();++i)
   {
      const object_localization::detected_object_info &data = msg->info[i];
      std::string name_ = data.name;
      float  Ym         = data.Y_min;
      float  Zm         = data.Z_min;
      float  Xc         = data.X;
      float  Yc         = data.Y;
      float  Zc         = data.Z;
      float  d          = data.distance;
      float  A          = data.area;
      int    d_count    = data.detection_count;
      dobj.object.push_back({name_.c_str(),Ym,Zm,Xc,Yc,Zc,d,A,d_count});
      
      // ROS_INFO("%s center is @ (%f,%f) ",data.Class.c_str(),X_c,Y_c);     
     
   
   }


}

void state_cb(const mavros_msgs::State::ConstPtr & msg) 
{
  current_state = * msg;
}

void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr & msg) {
  local_x = msg->pose.position.x;
  local_y = msg->pose.position.y;
  local_z = msg-> pose.position.z;
}

void inertial_cb(const sensor_msgs::Imu::ConstPtr& msg)
{
    tf::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    yaw_ornt = yaw;

    }

int main(int argc, char ** argv) 
{
  ros::init(argc, argv, "auto_takeoff_land");
  ros::NodeHandle nh;


  ros::Subscriber state_sub = nh.subscribe < mavros_msgs::State >("mavros/state", 100, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe < geometry_msgs::PoseStamped >("mavros/local_position/pose", 100, local_position_cb);
  ros::Subscriber inertial_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data",100,inertial_cb);
  pose_sp_pub = nh.advertise < geometry_msgs::PoseStamped >("mavros/setpoint_position/local", 100);
  ros::ServiceClient arming_client = nh.serviceClient < mavros_msgs::CommandBool >("mavros/cmd/arming");
  ros::ServiceClient land_client = nh.serviceClient < mavros_msgs::CommandTOL >("mavros/cmd/land");
  ros::ServiceClient set_mode_client = nh.serviceClient < mavros_msgs::SetMode >("mavros/set_mode");
  ros::Subscriber subobj         = nh.subscribe<object_localization::detected_object>("/detected_object/info", 100, detected_object_cb);
  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(50.0);
 
  // wait for FCU connection
  while (ros::ok() && !current_state.connected) 
  {
    ros::spinOnce();
    rate.sleep();
  }

  WXc = 0;
  WYc = 0;
  WZc = 0;
  
  pose.pose.position.x = WXc;
  pose.pose.position.y = WYc;
  pose.pose.position.z = WZc;

  //send a few setpoints before starting
  for (int i = 0; ros::ok() && i < 10 * 20; ++i) 
  {
    pose_sp_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  mavros_msgs::CommandTOL land_cmd;
  land_cmd.request.yaw = 0;
  land_cmd.request.latitude = 0;
  land_cmd.request.longitude = 0;
  land_cmd.request.altitude = 0;

  ros::Time last_request = ros::Time::now();

  while (ros::ok() && !current_state.armed) 
    {
      if (current_state.mode != "OFFBOARD" &&(ros::Time::now() - last_request > ros::Duration(5.0))) 
      {
      if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) 
      {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } 
    else 
    {
      if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))) 
      {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success) 
        {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }

    pose_sp_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  count = 0;
  if(count ==0)
  {
    yawt = yaw_ornt;
  }
  ros::Time begin = ros::Time::now();
// Takeoff -----------
  while (ros::ok() && local_z < takeoff_alt) 
  {
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = takeoff_alt+0.3;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yawt);
    pose_sp_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
    count=1;
  }
 // Window searching -------------------- 
 while((ros::Time::now()- begin)<ros::Duration(60.0)) 
 {
   for (int j =0; j< dobj.object.size(); ++j)
   {
     std::string s = dobj.object[j].obj_name.c_str();
     detect_count =  dobj.object[j].detection_count;
     if (s.compare(target_name)==0 && detect_count <100)
     {
        WXc = dobj.object[j].X;
        WYc = dobj.object[j].Y;
        WZc = dobj.object[j].Z;
     }
   }
   publish_pos_sp(rate);

  }
  // Window mission
  if (detect_count >= 90)
  {
    while(abs(local_z-WZc)>0.3)
    {
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "map";
      pose.pose.position.x = 0.0;
      pose.pose.position.y = 0.0;
      pose.pose.position.z = WZc;

      pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yawt);
      publish_pos_sp(rate);
      ROS_INFO("Aligning vertically @ %f",WZc);
    }
    while(abs(local_y-WYc)>0.3)
    {
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "map";
      pose.pose.position.x = 0.0;
      pose.pose.position.y = WYc;
      pose.pose.position.z = WZc;
      pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yawt);
      publish_pos_sp(rate);
      ROS_INFO("Aligning sideway @ %f",WYc);
    } 
   while(abs(local_x-WXc)>0.3)
    {
      pose.header.stamp = ros::Time::now();
      pose.header.frame_id = "map";
      pose.pose.position.x = WXc;
      pose.pose.position.y = WYc;
      pose.pose.position.z = WZc;
      publish_pos_sp(rate);
      ROS_INFO("approaching window @ %f",WXc);
    }
    publish_pos_sp(rate);
  }

//Landing
 ROS_INFO_ONCE("LANDING ...");
            
  while (!(land_client.call(land_cmd) && land_cmd.response.success)) 
  {
    pose_sp_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

//subroutine
void publish_pos_sp(ros::Rate r)
{
  pose_sp_pub.publish(pose);
  ros::spinOnce();
  r.sleep(); 
}