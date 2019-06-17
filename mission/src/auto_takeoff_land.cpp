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

int count=0;
float takeoff_alt = 2.0;
float local_x;
float local_y;
float local_z;
float yaw_ornt;
float yawt;       

mavros_msgs::State current_state;

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


  ros::Subscriber state_sub = nh.subscribe < mavros_msgs::State >
    ("mavros/state", 100, state_cb);


  //cv::destroyWindow("view");
  ros::Subscriber local_pos_sub = nh.subscribe < geometry_msgs::PoseStamped >
    ("mavros/local_position/pose", 100, local_position_cb);


  ros::Subscriber inertial_sub = nh.subscribe<sensor_msgs::Imu>
    ("mavros/imu/data",100,inertial_cb);

  ros::Publisher local_pos_pub = nh.advertise < geometry_msgs::PoseStamped >
    ("mavros/setpoint_position/local", 100);

  // ros::Publisher angu_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
  //        ("mavros/setpoint_attitude/cmd_vel", 1000);

  ros::ServiceClient arming_client = nh.serviceClient < mavros_msgs::CommandBool >
    ("mavros/cmd/arming");

  ros::ServiceClient land_client = nh.serviceClient < mavros_msgs::CommandTOL >
    ("mavros/cmd/land");

  ros::ServiceClient set_mode_client = nh.serviceClient < mavros_msgs::SetMode >
    ("mavros/set_mode");

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(50.0);

  // wait for FCU connection
  while (ros::ok() && !current_state.connected) 
  {
    ros::spinOnce();
    rate.sleep();
  }

  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  pose.pose.position.z = 0;

  //send a few setpoints before starting
  for (int i = 0; ros::ok() && i < 10 * 20; ++i) 
  {
    local_pos_pub.publish(pose);
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

  while (ros::ok() && !current_state.armed) {
    if (current_state.mode != "OFFBOARD" &&
      (ros::Time::now() - last_request > ros::Duration(5.0))) {
      if (set_mode_client.call(offb_set_mode) &&
        offb_set_mode.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      last_request = ros::Time::now();
    } else {
      if (!current_state.armed &&
        (ros::Time::now() - last_request > ros::Duration(5.0))) {
        if (arming_client.call(arm_cmd) &&
          arm_cmd.response.success) {
          ROS_INFO("Vehicle armed");
        }
        last_request = ros::Time::now();
      }
    }

    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  count = 0;
  if(count ==0)
  {
    yawt = yaw_ornt;
  }
  while (ros::ok() && local_z < takeoff_alt) {
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "map";
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = takeoff_alt;
    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yawt);
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
    count=1;
  }

 for (int i = 0; ros::ok() && i < 300; ++i) 
  {
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

 ROS_INFO_ONCE("LANDING ...");
            
  while (!(land_client.call(land_cmd) &&
      land_cmd.response.success)) {
    local_pos_pub.publish(pose);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}