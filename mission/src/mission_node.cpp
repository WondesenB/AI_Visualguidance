#include "mission_node.h"

int search_pos= 0;
int search_dir = 1;
// subscriber callback function
void depth_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    // Get a pointer to the depth values casting the data
    // pointer to floating point
    float* depths = (float*)(&msg->data[0]);

    // Image coordinates of the center pixel
     u = msg->width / 2;
     v = msg->height / 2;

    // Linear index of the center pixel
    int centerIdx = u + msg->width * v;

    // Output the measure
    if(std::isfinite(depths[centerIdx]))
    {

    obstacle_front = 100.0*depths[centerIdx];
    //ROS_INFO("Front obstacle distance : %g m", depths[centerIdx]);
    }
}
void obstacle_dis_cb(const srf08_ranging::obstacle_distance::ConstPtr& msg)
{
   obstacle_up = msg->up;
   obstacle_right = msg->right;
   obstacle_left = msg->left;
  // ROS_INFO("Obstacle info, up: %d cm, right: %d cm, left: %d cm ",obstacle_up,obstacle_right,obstacle_left);

}

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

void camera_pos_cb(const geometry_msgs::PoseStamped::ConstPtr & msg) 
{
  Xcam = msg->pose.position.x;
  Ycam = msg->pose.position.y;
  Zcam = msg-> pose.position.z;
  
}
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
  ros::Subscriber state_sub = nh.subscribe < mavros_msgs::State >("mavros/state", 1000, state_cb);
  ros::Subscriber local_pos_sub = nh.subscribe < geometry_msgs::PoseStamped >("mavros/local_position/pose", 1000, local_position_cb);
  ros::Subscriber inertial_sub = nh.subscribe<sensor_msgs::Imu>("mavros/imu/data",1000,inertial_cb);
  ros::Subscriber subobj         = nh.subscribe<local_mapping::detected_object>("/detected_object/info", 100, detected_object_cb);
  ros::Subscriber local_vispos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 100,camera_pos_cb); //mocap vision_pose /mavros/vision_pose/pose
  ros::Subscriber obstacle_distance_sub = nh.subscribe<srf08_ranging::obstacle_distance>("/obstacle_distance/info", 100,obstacle_dis_cb);
  ros::Subscriber depth_sub               = nh.subscribe("/zed/zed_node/depth/depth_registered", 100, depth_callback);
  //publisher
  pose_sp_pub = nh.advertise < geometry_msgs::PoseStamped >("mavros/setpoint_position/local", 1000);
  //service
  ros::ServiceClient arming_client = nh.serviceClient < mavros_msgs::CommandBool >("mavros/cmd/arming");
  ros::ServiceClient land_client = nh.serviceClient < mavros_msgs::CommandTOL >("mavros/cmd/land");
  ros::ServiceClient set_mode_client = nh.serviceClient < mavros_msgs::SetMode >("mavros/set_mode");

  //the setpoint publishing rate MUST be faster than 2Hz
  ros::Rate rate(100.0);

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

  mission_type mission = mission_takeoff;
  track_window_cmd     win_tracking_cmd = search;
  window_tracking_state    win_tracking_state  =  searching;
  
  while (ros::ok() && !landing) 
  {

    switch(mission)
    {
    case mission_takeoff:
          landing = 0;
          Nwpx = 0.0;
          Nwpy = 0.0;
          Nwpz = takeoff_alt;
          //find_nextsafe_wp(Nwpx,Nwpy, Nwpz,mission,win_tracking_cmd,search_dir);
          pose_sp.header.stamp = ros::Time::now();
          pose_sp.header.frame_id = "map";
          pose_sp.pose.position.x = Nwpx;
          pose_sp.pose.position.y = Nwpy;
          pose_sp.pose.position.z = Nwpz+0.3 ;
          pose_sp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yawt);
          ROS_INFO("takeoff to altitude: %f ", takeoff_alt);
        if (local_z >= takeoff_alt)
        {
          ROS_INFO("Hovering ");
          wait_time(10,rate);
          mission = mission_wall;
          //mission = mission_window;
        }

        publish_pos_sp(rate); 
        count = 1;
        break;
    case mission_wall:
          landing = 0;
          Nwpx = 0.0;
          Nwpy = -1.0;
          Nwpz = takeoff_alt;    
          while(abs(local_y-Nwpy)>0.3)
          {
            find_nextsafe_wp(Nwpx,Nwpy, Nwpz,mission,win_tracking_cmd,search_dir);
            pose_sp.header.stamp = ros::Time::now();
            pose_sp.header.frame_id = "map";
            pose_sp.pose.position.x = Nwpx;
            pose_sp.pose.position.y = Nwpy;
            pose_sp.pose.position.z = Nwpz;
            pose_sp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yawt);
            publish_pos_sp(rate);
            ROS_INFO("looking for wall passing  path");
          } 
          Nwpx = 2.0;
          while(abs(local_x-Nwpx)>0.3)
          {
          
            find_nextsafe_wp(Nwpx,Nwpy, Nwpz,mission,win_tracking_cmd,search_dir);
            pose_sp.header.stamp = ros::Time::now();
            pose_sp.header.frame_id = "map";
            pose_sp.pose.position.x = Nwpx;
            pose_sp.pose.position.y = Nwpy;
            pose_sp.pose.position.z = Nwpz;
            pose_sp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yawt);
            publish_pos_sp(rate);
            ROS_INFO("passing wall");
          }
          Nwpy = 0.0;
          while(abs(local_y-Nwpy)>0.3)
          {
            
            find_nextsafe_wp(Nwpx,Nwpy, Nwpz,mission,win_tracking_cmd,search_dir);
            pose_sp.header.stamp = ros::Time::now();
            pose_sp.header.frame_id = "map";
            pose_sp.pose.position.x = Nwpx;
            pose_sp.pose.position.y = Nwpy;
            pose_sp.pose.position.z = Nwpz;
            pose_sp.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, yawt);
            publish_pos_sp(rate);
            ROS_INFO("searching for window");
            if (abs(local_y-Nwpy)<0.4)
            {
             mission = mission_window; 
            }
          }
          if (abs(local_y-Nwpy)<0.3)
          {
           mission = mission_window; 
          }                                        
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
                   if(start_timer == 1)
                   {
                    begin = ros::Time::now();
                    start_timer = 0;
                   }
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
                   publish_pos_sp(rate);  
                   if ((ros::Time::now()- begin)>ros::Duration(90.0))
                    {   
                       ROS_INFO("Time is over, changing to auto landing mode ");
                       mission = mission_landing;
                       break;
                    }
                        
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
                       wait_time(10, rate);
                       mission = mission_landing;
                       break;
                      }
                  }

                 if(abs(local_x - WXm) < 0.3)
                  {
                   wait_time(10, rate);
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


//--------- subroutines ----------

void wait_cycle(int cycle, ros::Rate r )
{
 for (int i=0; i<cycle; ++i)
  {
   pose_sp_pub.publish(pose_sp);
   ros::spinOnce();
   r.sleep(); 
  } 
}
void wait_time(float time, ros::Rate r )
{
 ros::Time start = ros::Time::now();
  while((ros::Time::now()- start)<ros::Duration(time))
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


void publish_pos_sp(ros::Rate r)
{
  pose_sp_pub.publish(pose_sp);
  ros::spinOnce();
  r.sleep(); 
}


void find_nextsafe_wp (float& wpx, float& wpy, float& wpz, mission_type mission, track_window_cmd& win_cmd ,int& d)
{
 
 switch(mission)
 {
  case mission_takeoff: 
       if (obstacle_up <= 100)
       {
        wpx = 0.0;
        wpy = 0.0;
        wpz = local_z-0.3;
       }
       else
       {
        wpx = wpx;
        wpy = wpy;
        wpz = wpz;
       }
       break;
  case mission_window: 
       if((win_cmd == search && d ==1 ) && obstacle_left <=100) 
       {
        d = -1;
       }
       if((win_cmd == search && d == -1 ) && obstacle_right <=100) 
       {
        d = 1;
       } 
      if((win_cmd == search ) && obstacle_front <=100) 
       {
        wpx = local_x-0.3;
       }
      if((win_cmd == search ) && obstacle_down <=100) 
       {
        wpz = local_z + 0.5;
       } 
      if((win_cmd == search ) && obstacle_up <=100) 
       {
        wpz = local_z - 0.5;
       } 
       if((win_cmd == align) && (obstacle_up <=100 || obstacle_right <=100  || obstacle_left <=100))  
       {
        wpx = local_x;
        wpy = local_y;
        wpz = local_z;  
       }
      if((win_cmd == approach) && (obstacle_front <=100 ))  
       {
        wpx = local_x;
        wpy = local_y;
        wpz = local_z;  
       }                           
       break;
  case mission_pole:     
       break; 
  case mission_pipe:     
       break;             
 }


}