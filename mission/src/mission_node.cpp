#include "mission_node.h"

geometry_msgs::PoseStamped pose_sp;
ros::Publisher pose_sp_pub;
struct windows_location
{
  std::vector<float> win_Xc;
  std::vector<float> win_Yc;
  std::vector<float> win_Zc;
  
}win_loc;

float WXm, WYm, WZm;
float Xcam, Ycam, Zcam;
// subscriber callback function
void detected_object_cb(const local_mapping::detected_object::ConstPtr&  msg)
{
   
 
  dobj.object.clear();
  for (int i =0; i<msg->info.size();++i)
   {

      const local_mapping::detected_object_info &data = msg->info[i];
      std::string name_ = data.name;
      float  Ym         = data.Y_min;
      float  Zm         = data.Z_min;
      float  Xc         = data.X;
      float  Yc         = data.Y;
      float  Zc         = data.Z;
      float  d          = data.distance;
      float  A          = data.area;
      dobj.object.push_back({name_.c_str(),Ym,Zm,Xc,Yc,Zc,d,A});
      
      // ROS_INFO("%s center is @ (%f,%f) ",data.Class.c_str(),X_c,Y_c);     
     
   
   }


}

void state_cb(const mavros_msgs::State::ConstPtr & msg) 
{
  current_state = * msg;
}

void camera_pos_cb(const geometry_msgs::PoseStamped::ConstPtr & msg) 
{
  Xcam = msg->pose.position.x;
  Ycam = msg->pose.position.y;
  Zcam = msg-> pose.position.z;
  
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
  ros::Subscriber state_sub      = nh.subscribe < mavros_msgs::State >("mavros/state", 100, state_cb);
  ros::Subscriber local_pos_sub  = nh.subscribe < geometry_msgs::PoseStamped >("mavros/local_position/pose", 100, local_position_cb);
  ros::Subscriber inertial_sub   = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data",100,inertial_cb);
  ros::Subscriber subobj         = nh.subscribe<local_mapping::detected_object>("/detected_object/info", 10, detected_object_cb);
  ros::Subscriber local_vispos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10,camera_pos_cb); //mocap vision_pose /mavros/vision_pose/pose
  //publisher
   pose_sp_pub = nh.advertise < geometry_msgs::PoseStamped >("mavros/setpoint_position/local", 100);
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

  
  pose_sp.pose.position.x = 0;
  pose_sp.pose.position.y = 0;
  pose_sp.pose.position.z = 0;
  pose_sp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw_ornt);

  //send a few setpoints before starting
  for (int i = 0; ros::ok() && i < 10 * 20; ++i) 
  {
    pose_sp_pub.publish(pose_sp);
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

    pose_sp_pub.publish(pose_sp);
    ros::spinOnce();
    rate.sleep();
  }
  
  if (count == 0)
  {
    yawt = yaw_ornt;
  }

  mission_type        mission  = mission_takeoff;
  track_window_cmd     win_tracking_cmd = window_search;
  window_tracking_state    win_tracking_state  =  window_searching;
  
  while (ros::ok() && !landing) 
  {

    switch(mission)
    {
    case mission_takeoff:
          landing = 0;
          pose_sp.header.stamp = ros::Time::now();
          pose_sp.header.frame_id = "map";
          pose_sp.pose.position.x = 0;
          pose_sp.pose.position.y = 0;
          pose_sp.pose.position.z = takeoff_alt+0.3 ;
          pose_sp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yawt);
          ROS_INFO("takeoff to altitude: %f ", takeoff_alt);
        if (local_z >= takeoff_alt)
        {
          ROS_INFO("Hovering ");
          wait_cycle(300,rate);
          mission = mission_window;
        }

        pose_sp_pub.publish(pose_sp);
        ros::spinOnce();
        rate.sleep(); 
        count = 1;
        break;
    case mission_window:
          // pass through window

          WXm = 0.0;
          WYm = 0.0;
          WZm = Zcam;
          limitWin_location(WXm, WYm, WZm);
          pose_sp.header.stamp = ros::Time::now();
          pose_sp.header.frame_id = "map";
          pose_sp.pose.position.x = WXm;
          pose_sp.pose.position.y = WYm;
          pose_sp.pose.position.z = WZm ;
          ROS_INFO("Aligning vertically ");
          wait_cycle(100,rate);
          if (abs(Ycam) > 0.5 )
          {
          mission = mission_landing;  
          }
         break;
    case mission_landing:
                  
          ROS_INFO("LANDING");
          while (!(land_client.call(land_cmd) && land_cmd.response.success)) 
          {
            pose_sp_pub.publish(pose_sp);
            ros::spinOnce();
            rate.sleep();
                        
          }
          landing = 1;
                
          break;
    }
  }

                     
  return 0;
}


void wait_cycle(int cycle, ros::Rate r )
{
 for (int i=0; i<cycle; ++i)
  {
   pose_sp_pub.publish(pose_sp);
   ros::spinOnce();
   r.sleep(); 
  } 
}


void limitWin_location(float& x, float& y, float& z)
{
  if(x >7.0 || x < 2.0)
  {
    x = Xcam;
  }
  else
  {
    x = x;
  }

  if(abs(abs(y)-abs(local_y))> 10.0)
  {
    y = Ycam;
  }
  else
  {
    y =y;
  }

  if(z >2.5 || z < 1.0)
  {
    z = 1.0;
  }
  else
  {
    z = z;
  }
}