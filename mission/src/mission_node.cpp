#include "mission_node.h"

// subscriber callback function
void state_cb(const mavros_msgs::State::ConstPtr & msg) 
{
  current_state = * msg;
}

void local_position_cb(const geometry_msgs::PoseStamped::ConstPtr & msg) 
{
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

// Main 

int main(int argc, char ** argv) 
{
  ros::init(argc, argv, "mission_node");
  ros::NodeHandle nh;
  // subscriber
  ros::Subscriber state_sub = nh.subscribe < mavros_msgs::State >("mavros/state", 100, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe < geometry_msgs::PoseStamped >("mavros/local_position/pose", 100, local_position_cb);
  ros::Subscriber inertial_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data",100,inertial_cb);
  //publisher
  ros::Publisher pose_sp_pub = nh.advertise < geometry_msgs::PoseStamped >("mavros/setpoint_position/local", 100);
  //service
  ros::ServiceClient arming_client = nh.serviceClient < mavros_msgs::CommandBool >("mavros/cmd/arming");
  ros::ServiceClient land_client = nh.serviceClient < mavros_msgs::CommandTOL >("mavros/cmd/land");
  ros::ServiceClient set_mode_client = nh.serviceClient < mavros_msgs::SetMode >("mavros/set_mode");

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
  pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw_ornt);

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

// Checking offboard mode and arming
  while (ros::ok() && !current_state.armed) 
  {
    if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))) 
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
  
  if (count == 0)
  {
    yawt = yaw_ornt;
  }

  mission_type mission = mission_takeoff;

  
  while (ros::ok() && !landing) 
  {

    switch(mission)
    {
    case mission_takeoff:
          landing = 0;
          pose.header.stamp = ros::Time::now();
          pose.header.frame_id = "map";
          pose.pose.position.x = 0;
          pose.pose.position.y = 0;
          pose.pose.position.z = takeoff_alt+0.3 ;
          pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yawt);
          ROS_INFO("takeoff to altitude: %f ", takeoff_alt);
        if (local_z >= takeoff_alt)
        {
          for (int i=0; i<300; ++i)
          {
           ROS_INFO("Hovering ");
           pose_sp_pub.publish(pose);
           ros::spinOnce();
           rate.sleep(); 
          }
         mission = mission_landing;
        }

        pose_sp_pub.publish(pose);
        ros::spinOnce();
        rate.sleep(); 
        count = 1;
        break;
    case mission_window:
          // pass through window
         break;
    case mission_landing:
           
          pose.header.stamp = ros::Time::now();
          pose.header.frame_id = "map";
          pose.pose.position.x = 0;
          pose.pose.position.y = 0;
          pose.pose.position.z = 0 ;   
          pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw_ornt);             
          ROS_INFO("LANDING");
          while (!(land_client.call(land_cmd) && land_cmd.response.success)) 
          {
            pose_sp_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
                        
          }
          landing = 1;
                
          break;
    }
  }

                     
  return 0;
}
