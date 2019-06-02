#include "mission_node.h"

int search_pos= 0;
int search_dir = 1;
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

  ros::Time begin = ros::Time::now();
  ROS_INFO("time duration in secs = %f", (ros::Time::now()- begin)*1e-9);

  mission_type        mission  = mission_takeoff;
  track_window_cmd     win_tracking_cmd = search;
  window_tracking_state    win_tracking_state  =  searching;
  
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

        publish_pos_sp(rate); 
        count = 1;
        break;
    case mission_window:
          // pass through window
          landing = 0;
          switch(win_tracking_cmd)
          {
            case search:
                 win_tracking_state = searching;
                 ROS_INFO("searching");
                 while (win_tracking_cmd == search)
                 {
                   for (int j =0; j< dobj.object.size(); ++j)
                   {
                      std::string s = dobj.object[j].obj_name.c_str();
                     if (s.compare(target_name)==0)
                     {
                        wins.wndw.clear();
                        wins.wndw.push_back({dobj.object[j].X,dobj.object[j].Y,dobj.object[j].Z});
                        ROS_INFO("Last %s  found @ (%f, %f, %f)",dobj.object[j].obj_name.c_str(), dobj.object[j].X,dobj.object[j].Y,dobj.object[j].Z);
                        win_tracking_cmd = align;
                        break;

                     }
                     publish_pos_sp(rate);
                   } 
                   window_scan (search_pos,search_dir,yawt,rate,win_tracking_cmd);
                   publish_pos_sp(rate);                
                 }

                  publish_pos_sp(rate);
                  break;

            case align:
                 //window center
                 win_tracking_state = aligning;
                 
                 if(wins.wndw.size()>=1)
                 {
                   WXm = 0.0;
                   WYm = wins.wndw[0].win_Yc ;
                   WZm = wins.wndw[0].win_Zc ;
                   limitWin_location(WXm, WYm, WZm);
                   ROS_INFO("Aligning");
                 }
                 else
                 {
                   win_tracking_cmd = search; 
                   break;
                 }
                 if (abs(local_z-WZm)>0.3 || abs(local_y - WYm) > 0.3)
                 {

                  while(abs(local_z-WZm)>0.3)
                  {
                    pose_sp.header.stamp = ros::Time::now();
                    pose_sp.header.frame_id = "map";
                    pose_sp.pose.position.x = WXm;
                    pose_sp.pose.position.y = 0.0;
                    pose_sp.pose.position.z = WZm;
                    pose_sp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yawt);
                    publish_pos_sp(rate);
                    ROS_INFO("Aligning vertically @ %f",WZm);
                  }
                  while(abs(local_y-WYm)>0.3)
                  {
                    pose_sp.header.stamp = ros::Time::now();
                    pose_sp.header.frame_id = "map";
                    pose_sp.pose.position.x = WXm;
                    pose_sp.pose.position.y = WYm;
                    pose_sp.pose.position.z = WZm;
                    pose_sp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yawt);
                    publish_pos_sp(rate);
                    ROS_INFO("Aligning sideway @ %f",WYm);
                  } 
                  publish_pos_sp(rate);
                  if(abs(local_z-WZm)<0.3 && abs(local_y - WYm) < 0.3)
                   { 
                   win_tracking_cmd = approach; //search
                   }

                 }
                 else
                 {
                  publish_pos_sp(rate);
                  win_tracking_cmd = approach; //search
                 }
            			/*if ((ros::Time::now()- begin)>ros::Duration(20.0))
            			  {
            			   mission = mission_landing;
            			  }
            			 */

                break;

            case approach:
                 win_tracking_state = approaching;
                 ROS_INFO("approaching");
                 if(wins.wndw.size()>=1)
                 {
                   WXm = wins.wndw[0].win_Xc ;
                   WYm = wins.wndw[0].win_Yc ;
                   WZm = wins.wndw[0].win_Zc ;
                   limitWin_location(WXm, WYm, WZm);
                 }
                 else
                 {
                   win_tracking_cmd = search; 
                   break;
                 }
                while(abs(local_x-WXm)>0.3)
                  {
                    pose_sp.header.stamp = ros::Time::now();
                    pose_sp.header.frame_id = "map";
                    pose_sp.pose.position.x = WXm;
                    pose_sp.pose.position.y = WYm;
                    pose_sp.pose.position.z = WZm;
                    publish_pos_sp(rate);
                    ROS_INFO("approaching window @ %f",WXm);
                    if(abs(local_x - WXm) < 0.3)
                      {
                       wait_cycle(300, rate);
                       mission = mission_landing;
                       break;
                      }
                  }

                 if(abs(local_x - WXm) < 0.3)
                  {
                   wait_cycle(300, rate);
                   mission = mission_landing;
                   publish_pos_sp(rate);                   
                   break;
                  }                 
                  publish_pos_sp(rate);
                break;
          }

         break;
    case mission_landing:
                  
          ROS_INFO("LANDING");
          while (!(land_client.call(land_cmd) && land_cmd.response.success)) 
          {
          publish_pos_sp(rate);
                        
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
  //---x --
  if(abs(x-local_x) >10.0 )
  {
    x = 0;
  }
  else if (abs(x-local_x) >7.0 )
  {
    x = 5;
  }
  else if (abs(x-local_x) <1.5 )
  {
    x = local_x;
  }
  else
  {
    x = 0.5*x;
  }

// --y--
  if((y-local_y)> 10.0)
  {
    y = 10.0;
  }
  else if((y-local_y)< -10.0)
  {
    y =-10.0;
  }
  else
  {
    y = y;
  }

// --z --
  if(z >3.0 )
  {
    z = 2;
  }
  else if(z < 1.0)
  {
    z= 1.0;
  }
  else
  {
    z = z;
  }
}

void winlocation_stat(windows wds, float& Wxm, float& Wym, float& Wzm, float& Xt,float& Yt, float& Zt, float& Xvar, float& Yvar,float& Zvar )
{

}
void publish_pos_sp(ros::Rate r)
{
  pose_sp_pub.publish(pose_sp);
  ros::spinOnce();
  r.sleep(); 
}

void window_scan ( int& p, int& d ,float yaw,ros::Rate r, track_window_cmd& window_cmd )
{ 
  float x =0;
  float y = 0;
  float z = 1.5;
  if (p == 0 && d== 1)
  {
    y = 1.0f;
  }
  else if (p == 0 && d== -1)
  {
    y = -1.0f;
  }
 else if (p == 1 && d== 1)
  {
    y = 2.0f;
  }    
 else if (p == 1 && d== -1)
  {
    y = 0.0f;
  }
 else if (p == -1 && d== 1)
  {
    y = 0.0f;
  }
 else if (p == -1 && d== -1)
  {
   y = -2.0f;
  } 
 else if (p == 2 && d== 1)
  {
    y = 2.0f;
  }
 else if (p == 2 && d== -1)
  {
    y = 1.0f;
  }
 else if (p == -2 && d== -1)
  {
   y = -2.0f;
  } 
  else if (p == -2 && d== 1)
  {
   y = -1.0f;
  } 
  else
  {
    y = 0.0f;
  }   
  pose_sp.header.stamp = ros::Time::now();
  pose_sp.header.frame_id = "map";
  pose_sp.pose.position.x = x;
  pose_sp.pose.position.y = y;
  pose_sp.pose.position.z = z ;
  pose_sp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yaw); 
  publish_pos_sp(r); 
  
  if (p == 0 && d == 1 && (abs(local_y-1.0)<0.3))
  {
    pause4search(100,r,window_cmd);
    p = 1;
    d = 1;
  }
  else if (p == 0 && d == -1 && (abs(local_y+1.0)<0.3))
  {
    pause4search(100,r,window_cmd);
    p = -1;
    d = -1;
  } 
  else if (p == 1 && d == 1 && (abs(local_y-2.0)<0.3))
  {
    pause4search(100,r,window_cmd);
    p = 2;
    d = -1;
  }
 else if (p == 1 && d == -1 && (abs(local_y)<0.3))
  {
    pause4search(100,r,window_cmd);
    p = 0;
    d = -1;
  } 
 else if (p == -1 && d == 1 && (abs(local_y)<0.3))
  {
    pause4search(100,r,window_cmd);
    p = 0;
    d = 1;
  } 
  else if (p == -1 && d == -1 && (abs(local_y+2.0)<0.3))
  {
    pause4search(100,r,window_cmd);
    p = -2;
    d = 1;
  } 
  else if (p == 2 && d == -1 && (abs(local_y-1.0)<0.3))
  {
    pause4search(100,r,window_cmd);
    p = 1;
    d = -1;
  } 
  else if (p == -2 && d == 1 && (abs(local_y+1.0)<0.3))
  {
    pause4search(100,r,window_cmd);
    p = -1;
    d = 1;
  } 
 else if ( (local_y-2.0)>0.3)
  {
    pause4search(100,r,window_cmd);
    p = 2;
    d = -1;
  } 
 else if ((local_y+2.0)< -0.3)
  {
    pause4search(100,r,window_cmd);
    p = -2;
    d = 1;
  } 
  else
  {
    pause4search(100,r,window_cmd);
    p = p;
    d = d;
  }
  publish_pos_sp(r);        
}

void pause4search(int cycle, ros::Rate r , track_window_cmd& win_cmd )
{
  int found = 0;

 for (int i=0; i<cycle; ++i)
  {
     pose_sp_pub.publish(pose_sp);
     ros::spinOnce();
     r.sleep(); 
     for (int j =0; j< dobj.object.size(); ++j)
     {
        std::string s = dobj.object[j].obj_name.c_str();
       if (s.compare(target_name)==0)
       {
          wins.wndw.clear();
          wins.wndw.push_back({dobj.object[j].X,dobj.object[j].Y,dobj.object[j].Z});
          ROS_INFO("Last %s  found @ (%f, %f, %f)",dobj.object[j].obj_name.c_str(), dobj.object[j].X,dobj.object[j].Y,dobj.object[j].Z);
          win_cmd = align;
          found =1;
          break;
       }
     }
    if(found == 1)
    {
      break;
    }    
  }
} 