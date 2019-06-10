#include "old_pathfinder_node.h"
float local_x;
float local_y;
float local_z;
float yaw_ornt;
int boxNM;    //number of detected bounding boxes / objects
 

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
  current_state = *msg;
}

 void local_pos_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
   local_x = msg->pose.position.x;
   local_y = msg->pose.position.y;
   local_z = msg->pose.position.z;   

}

void imu_cb(const sensor_msgs::Imu::ConstPtr& msg)
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


void BoundBox_info_cb(const local_mapping::detected_object::ConstPtr& msg)
{
   
    ob.obj.clear();
  //if(boxID->obj[0].obj_type != "")
  //{
     boxNM = msg->info.size();
   //ROS_INFO("object size is:%s",msg->info[0].name.c_str());
   // BoundBox *boxID;    
    // boxID = new BoundBox[boxNM];

    for(int i = 0; i < boxNM; i++)
    {
      //const zed_data_subscribe::detected_object &data = msg->info[i];
      const local_mapping::detected_object_info &data = msg->info[i];
      string  obj_type = data.name.c_str();
      float   y_min = data.Y_min;
      float   y_max = 2.0*data.Y - y_min;
      float   z_min = data.Z_min;
      float   x_cnt = data.X;
      float   y_cnt = data.Y;
      float   z_cnt = data.Z;
      //float   dist = data.distance;
      ob.obj.push_back({obj_type, y_min, y_max, z_min, x_cnt, y_cnt, z_cnt});
     
    }
 
    
}



int main( int argc, char** argv ) {

    ros::init(argc, argv, "old_pathfinder_node");
  const float ALT_LIMIT = 3.0;
    float yawt;
    int count; 
    int increm;   
  const float PI = 3.14159265359;
  const float TOLRNCE = 0.2;       //acceptance radius
  const float OBJ_CLR = 1.0;    //object clearance to avoid colission
  const float ALT_SAFETY = 0.65;     //maintain safe altitude to avoid roof collision
  //const float ROLL_SAFETY = 0.75;
    bool obj_detected = false;
  const float MARGIN = 0.5;
    float wall_dist;
    float chk_target_x;
    float chk_target_y;

    float rot_r ;
    float rot_l = 0.0;
  const float ROT_MAX = PI/2.0;
  const float ROT_MIN = -1.75*PI;
  const float COEF = 0.0015;     //attitude rate controller
    
    float target_x;
    float target_y;
    float target_z;

    float depth1;
    float depth2;
    
    float coefy; //y increment rate control
    float coefz; //Z increment rate control
                
    //vector <string> object_list;
  //const float UAV_HEIGHT = 1.0;
  const float UAV_WIDTH = 1.5;
  const float DEPTH_DIF = 3.0; //depth difference b/n two consequetive bounding boxes
  const float MN_FOBT_D =  1.5; //minimum front obstacle distance
  const float MN_SOBT_D =  1.25; //minimum side obstacle distance
    ros::NodeHandle nh;

  
  //subscribers
  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
        ("mavros/imu/data",100, imu_cb);

  ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>
        ("mavros/local_position/pose", 100, local_pos_cb);

  ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 100, state_cb);

  ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
        ("mavros/cmd/land");

  ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
        ("mavros/cmd/arming");

  ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
        ("mavros/set_mode");
  
  
  ros::Subscriber  BoundBox_sub = nh.subscribe<local_mapping::detected_object> 
        ("/detected_object/info", 1000, &BoundBox_info_cb);
 
  ros::Publisher local_setpos_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/mavros/setpoint_position/local", 1000); 


        PathFinder old_pathfinder_node;
          //BoundBox Bbox;// = &boxID.obj;      //single object of class BoundBox
          
         // BoundBox Bbox;//= boxID[1];
        //BoundBox *boxID = &boxID.obj;
        //objects *boxID;// = &ob;
        //boxID = new objects[4];
        //objects *boxID = new objects;


       // objects* boxID = new objects[boxNM]();

        objects* boxID = &ob;
      /*
       //OR
        objects *boxID;
        boxID = new objects[boxNM];
      */   

    ros::Rate rate(100.0);
    //send fake position setpoints to initialize communication
    geometry_msgs::PoseStamped pose;

    while(ros::ok() && !current_state.connected)
    { 
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = "map"; 
        pose.pose.position.x = 0;
        pose.pose.position.y = 0;
        pose.pose.position.z = 0;
        
        local_setpos_pub.publish(pose);             
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

 while(ros::ok() && !current_state.armed){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

       local_setpos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
  

//START NAVIGATION

count = 0;
while(local_z < 0.5*ALT_LIMIT)
  { 
    if(count == 0)
    {
      yawt = yaw_ornt;
    }
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "map";          
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 0.5*ALT_LIMIT + MARGIN; //COEF*count; //   to be omited when camera is mounted
  ROS_INFO_ONCE("Takeoff to altitude of :%f",0.5*ALT_LIMIT);  
  pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yawt);          
  local_setpos_pub.publish(pose);
  ros::spinOnce();
  rate.sleep();
  count++;
}


while(ros::ok() && offb_set_mode.request.custom_mode == "OFFBOARD" )
{ 
      //iterate through list of objects and compare with the detected object
        string none = "none";
        string wall = "wall";
        string window = "window";
        string pole = "pole";
        string pipe = "pipe";
        string tree = "tree";
        string net = "net";
        string land = "land";

       
    
        for(int i = 0; i < old_pathfinder_node.object_list.size(); ++i)
        {
          if(old_pathfinder_node.object_list[i] == boxID->obj[0].obj_type && i == 0)
          //if(old_pathfinder_node.object_list[i] == window && i == 0)
          {
            ROS_INFO("NEW MISSION CHECK...");
            obj_detected = true;
          }
          
        }

        for (const auto& value : old_pathfinder_node.object_list)
          {
              std::cout <<value << '\n';
          } 
    
    //******************************************************************************    
    //            MISSION WALL
    //
    //******************************************************************************
   
    if(obj_detected && boxID->obj[0].obj_type == wall)
    { 
       ROS_INFO_ONCE("MISSION SEGMENT :%s ",boxID->obj[0].obj_type.c_str());
            
       float RL_ymax = -5.0;   
       float LL_ymin = 5.0; 

      if(boxID->obj[0].y_cnt >= 0)
      {
        target_y = 0.5*( RL_ymax + boxID->obj[0].y_min);
      }else if(boxID->obj[0].y_cnt < 0)
      {
        target_y = 0.5*(LL_ymin + boxID->obj[0].y_max);
      }
        
      while(abs(target_y - local_y) > TOLRNCE)
      {            
         pose.header.stamp = ros::Time::now();                        
         pose.header.frame_id = "map";
         pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yawt);                   
         pose.pose.position.y =  target_y;                                         
         local_setpos_pub.publish(pose);         
         ROS_INFO_ONCE("Roll to passby the wall");
         ros::spinOnce();
         rate.sleep();
       
      }               


      target_x = boxID->obj[0].x_cnt + OBJ_CLR;

      while(local_x < target_x){      
          pose.header.stamp = ros::Time::now();                
          pose.header.frame_id = "map";
         pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yawt);
         pose.pose.position.x = target_x;// + COEF*count;
         pose.pose.position.y = target_y;
          pose.pose.position.z = pose.pose.position.z;                   
         ROS_INFO_ONCE("Passingby the wall");
         local_setpos_pub.publish(pose);
         ros::spinOnce();
         rate.sleep();
        
      }
      old_pathfinder_node.object_list.erase(old_pathfinder_node.object_list.begin()); 
      //old_pathfinder_node.object_list.resize(old_pathfinder_node.object_list.size() - 1);       
      obj_detected = false;
      //old_pathfinder_node.object_list.push_back(window);

      //maybe roll to search path can be added
      //ROS_INFO("MISSION COMPLETED:%s AT LOCAL_X = %f",boxID->obj[0].obj_type.c_str(), local_x);

    //****************************************************************************************    
    //                           MISSION WINDOW
    //
    //**************************************************************************************** 
           
    }else if(obj_detected && boxID->obj[0].obj_type == window)
    {        
        ROS_INFO_ONCE("MISSION SEGMENT :%s ",boxID->obj[0].obj_type.c_str());  

        float target_x = boxID->obj[0].x_cnt + OBJ_CLR;
        increm = 1;     //intervals to check/recheck target center
        count = 0;
        while(ros::ok() && local_x < target_x && count <= 1)            
        {

          target_y = boxID->obj[0].y_cnt;
          target_z = boxID->obj[0].z_cnt;
         
              while(abs(target_y - local_y) > TOLRNCE ||
                    abs(target_z - local_z) > TOLRNCE)
              {
                 pose.header.stamp = ros::Time::now();                  
                 pose.header.frame_id = "map";
                 pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yawt);                   
                 pose.pose.position.y = target_y;
                 pose.pose.position.z = target_z;                   
                 local_setpos_pub.publish(pose);
                 ROS_INFO_ONCE("Roll to head to:%s",boxID->obj[0].obj_type.c_str());
                 //ROS_INFO("local_x:%f, local y:%f, local_z:%f",local_x,local_y, local_z);
                  ros::spinOnce();
                  rate.sleep();
                  
              }  

             chk_target_x = 0.5*(target_x + local_x);
              while(local_x <= count*target_x + increm*chk_target_x)
              {
                 pose.header.stamp = ros::Time::now();                  
                 pose.header.frame_id = "map";
                 pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yawt);
                 pose.pose.position.x = count*target_x + increm*chk_target_x;
                 pose.pose.position.y = target_y;
                 pose.pose.position.z = target_z;                   
                 ROS_INFO_ONCE("Adance towards the:%s",boxID->obj[0].obj_type.c_str());
                 //ROS_INFO("target_x:%f local y:%f, local_x:%f",target_x, local_y, local_x);
                 local_setpos_pub.publish(pose);
                 ros::spinOnce();
                 rate.sleep();                     
               }
               //hover for a while
               for(int i = 0; i < 100; ++i)
               {
                pose.header.stamp = ros::Time::now();                  
                pose.header.frame_id = "map";
                local_setpos_pub.publish(pose);
                ROS_INFO_ONCE("HOVERING");
              ros::spinOnce();
              rate.sleep();
               }
                increm--;
                count++;
               //land at current location
               //# AUTO-LANDING
               /*
               ROS_INFO("LANDING...");
              while (!(old_pathfinder_node.land_client.call(land_cmd) &&
                  land_cmd.response.success)) {                    
                  pose.header.stamp = ros::Time::now();                  
                  pose.header.frame_id = "map";
                  local_setpos_pub.publish(pose);
                  ros::spinOnce();
                  rate.sleep();
              }
              */
               /*
              //# SETPOINT LANDING
              float land_z = -0.5;
               while(local_z >= land_z){
                 pose.header.stamp = ros::Time::now();                          
                 pose.header.frame_id = "map";
                 pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yawt);
                 pose.pose.position.x = pose.pose.position.x;               
                 pose.pose.position.y = pose.pose.position.y;
                 pose.pose.position.z = land_z;                       
                 local_setpos_pub.publish(pose);
                 ros::spinOnce();
                 rate.sleep();               
               } 
             */
            
        }

        old_pathfinder_node.object_list.erase(old_pathfinder_node.object_list.begin()); 
        //old_pathfinder_node.object_list.resize(old_pathfinder_node.object_list.size() - 1);
        obj_detected = false;          

    }else if(obj_detected && boxID->obj[0].obj_type.c_str() == pole)
    {
          
        //******************************************************************************    
        //            MISSION POLE
        //
        //******************************************************************************
       ROS_INFO_ONCE("MISSION SEGMENT :%s ",boxID->obj[0].obj_type.c_str());
             //ROS_INFO("bounding box number =%i",boxNM);
               
            float t_x;   //x-axis target distance between consequetive obstacles
            float t_y;   //y-axis target distance between consequetive obstacle
            int W_pathID;   //wide path id
            int N_pathID;    //narrow path id
            float W_min_d = 1000.0;  //initialize guess of minimum distance to temp_goal for wide path
            float N_min_d = 1000.0;  //initialize minimum distance to temp_goal for narrow path
            float temp_goal_x = 26.85; //intermidiate goal point
            float temp_goal_y = 2.00; //intermidiate goal point
            float RL_ymax = -5.0;   
            float LL_ymin = 5.0;  
            int a, b;             
            
            float target_d;   //distance between a target point and the temp_goal point
            
            while(ros::ok() && local_x <= temp_goal_x)
            {

              while(obj_detected && boxID->obj[0].obj_type.c_str() == pole)
              {
                           

               for(int i = 0; i < boxNM; ++i)
                { 
                  
                                       //BOUNDERY CONDITIONS: 
                  //=============================================================                                      
                                            if(i == 0)
                                            {
                                              a = 0;
                                              b = 1;
                                            }else
                                            {
                                              a = 1;
                                              b = 0;
                                            }    
                  //=============================================================
                  if(abs(boxID->obj[i].y_min - (a*boxID->obj[i-1*a].y_max + b*RL_ymax)) >= UAV_WIDTH)
                  {  

                    t_x  = max(boxID->obj[i].x_cnt, (a*boxID->obj[i-1*a].x_cnt + b*local_x));                     
                    t_y = 0.5*(boxID->obj[i].y_min + (a*boxID->obj[i-1*a].y_max + b*RL_ymax));
                    target_d = sqrt(pow((temp_goal_x - t_x),2) + pow((temp_goal_y - t_y),2));

                    if(W_min_d > target_d)
                    {
                      W_min_d = target_d;  //minimum distance to temp_goal through wide path
                      W_pathID = i;       //location of minimum distance to temp_goal through wide path             
                    }

                    //ROS_INFO("path obtained at W_pathID:%i",W_pathID);

                  }else if(abs(boxID->obj[i].y_min - (a*boxID->obj[i-1*a].y_max + b*RL_ymax)) < UAV_WIDTH &&
                    abs(boxID->obj[i].x_cnt - (a*boxID->obj[i-1*a].x_cnt + b*local_x)) >= DEPTH_DIF)
                  {
                    if(boxID->obj[i].x_cnt > (a*boxID->obj[i-1*a].x_cnt + b*local_x) && //true for right and false for left BCs.
                      abs((b*boxID->obj[i+1*b].y_min + a*LL_ymin) - (a*boxID->obj[i-1*a].y_max + b*RL_ymax)) >= 2.0*MN_SOBT_D)
                    {
                      t_y = (a*boxID->obj[i-1*a].y_max + b*RL_ymax) + MN_SOBT_D;
                      t_x = boxID->obj[i].x_cnt - MN_FOBT_D;

                    }else if(boxID->obj[i].x_cnt < (a*boxID->obj[i-1*a].x_cnt + b*local_x) && //false for right and true for left BCs.
                      abs(boxID->obj[i].y_min - (a*boxID->obj[i-2*a].y_max + b*RL_ymax)) >= 2.0*MN_SOBT_D)
                    {
                      t_y = boxID->obj[i].y_min - MN_SOBT_D;
                      t_x = (a*boxID->obj[i-1*a].x_cnt + b*local_x) - MN_FOBT_D;
                    }
                  target_d = sqrt(pow((temp_goal_x - t_x),2) + pow((temp_goal_y - t_y),2));
                //find the minimum path id
                    if(N_min_d > target_d)
                    {
                       N_min_d = target_d;
                       N_pathID = i;                    
                    }
                    //ROS_INFO("path obtained at N_pathID:%i",N_pathID);
                  }else if(abs(boxID->obj[i].y_min - (a*boxID->obj[i-1*a].y_max + b*RL_ymax)) < UAV_WIDTH &&
                    abs(boxID->obj[i].x_cnt - (a*boxID->obj[i-1*a].x_cnt + b*local_x)) < DEPTH_DIF)
                  {
                    ROS_INFO_ONCE("THERE IS NO WAY TO PASS. ATEMPT TO ROLL"); //add rolling condition
                  }
                  

                }

                //START NAVIGATING BASED ON THE ABOVE OPTIONS

               if(abs(boxID->obj[W_pathID].y_min - (a*boxID->obj[W_pathID-1*a].y_max + b*RL_ymax)) >= UAV_WIDTH)
                  {               
                    target_x = max(boxID->obj[W_pathID].x_cnt, (a*boxID->obj[W_pathID-1*a].x_cnt + b*local_x)) + OBJ_CLR;                 
                    target_y = 0.5*(boxID->obj[W_pathID].y_min + (a*boxID->obj[W_pathID-1*a].y_max + b*RL_ymax));
                    ROS_INFO_ONCE("W_pathid:%i,target_x:%f, target_y:%f",W_pathID,target_x, target_y);
                    
                        
                      while(abs(target_y - local_y) > TOLRNCE){                                               
                        //coefy = abs(target_y - local_y);
                        pose.header.stamp = ros::Time::now();                        
                        pose.header.frame_id = "map";
                       pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yawt); 
                       pose.pose.position.x = pose.pose.position.x;
                       pose.pose.position.y = target_y;  
                       pose.pose.position.z = pose.pose.position.z;
                       local_setpos_pub.publish(pose);
                       //ROS_INFO_ONCE("Pass through wide path");
                       //ROS_INFO("roll to wide path: local x:%f, local_y:%f",local_x, local_y);
                       ros::spinOnce();
                       rate.sleep();
                       
                       }                  
             
                       while(local_x <= target_x){
                         pose.header.stamp = ros::Time::now();                          
                         pose.header.frame_id = "map";
                         pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yawt);
                         pose.pose.position.x = target_x;                       
                         pose.pose.position.y = pose.pose.position.y;
                         pose.pose.position.z = pose.pose.position.z;
                        // ROS_INFO("advance:local_x:%f local_y:%f",local_x, local_y);
                         local_setpos_pub.publish(pose);
                         ros::spinOnce();
                         rate.sleep();                       
                       }

                  }else if(abs(boxID->obj[N_pathID].y_min - (a*boxID->obj[N_pathID-1*a].y_max + b*RL_ymax)) < UAV_WIDTH &&
                    abs(boxID->obj[N_pathID].x_cnt - (a*boxID->obj[N_pathID-1*a].x_cnt + b*local_x)) >= DEPTH_DIF)
                  {
                    if(boxID->obj[N_pathID].x_cnt > boxID->obj[N_pathID-1].x_cnt &&
                      abs((b*boxID->obj[N_pathID+1*b].y_min + a*LL_ymin) - (a*boxID->obj[N_pathID-1*a].y_max + b*RL_ymax)) >= 2.0*MN_SOBT_D)
                    {
                      target_y = (a*boxID->obj[N_pathID-1*a].y_max + b*RL_ymax) + MN_SOBT_D;
                      target_x = boxID->obj[N_pathID].x_cnt - MN_FOBT_D;
                      ROS_INFO_ONCE("LN_pathid:%i, tx:%f, ty:%f",N_pathID, target_x, target_y);
                    }else if(boxID->obj[N_pathID].x_cnt < (a*boxID->obj[N_pathID-1*a].x_cnt + b*local_x) &&
                      abs(boxID->obj[N_pathID].y_min - (a*boxID->obj[N_pathID-2*a].y_max + b*RL_ymax)) >= 2.0*MN_SOBT_D)
                    {
                      target_y = boxID->obj[N_pathID].y_min - MN_SOBT_D;
                      target_x = (a*boxID->obj[W_pathID-1*a].x_cnt + b*local_x) - MN_FOBT_D;
                      //ROS_INFO("RN_pathid:%i, tx:%f, ty:%f",N_pathID, target_x, target_y);
                    }
                       
                      while(abs(target_y - local_y) > TOLRNCE){                       
                        pose.header.stamp = ros::Time::now();                        
                        pose.header.frame_id = "map";
                        pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yawt);                   
                        pose.pose.position.x = pose.pose.position.x;
                        pose.pose.position.y =  target_y;  
                        pose.pose.position.z = pose.pose.position.z;                                          
                        local_setpos_pub.publish(pose);
                        ROS_INFO_ONCE("Right pass through narrow path");                     
                        ros::spinOnce();
                        rate.sleep();                     
                       }                  
                                
                       while(local_x <= target_x){
                         pose.header.stamp = ros::Time::now();                          
                         pose.header.frame_id = "map";
                         pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yawt);
                         pose.pose.position.x = target_x; 
                         pose.pose.position.y = pose.pose.position.y;
                         pose.pose.position.z = pose.pose.position.z;                       
                         local_setpos_pub.publish(pose);
                         ROS_INFO_ONCE("Advance between poles");
                         ros::spinOnce();
                         rate.sleep();                       
                       }            
                  }

                  //hover for a while
                  for(int i = 0; i < 100; ++i)
                     {
                      pose.header.stamp = ros::Time::now();                  
                      pose.header.frame_id = "map";
                      local_setpos_pub.publish(pose);
                      //ROS_INFO_ONCE("HOVERING");
                      ros::spinOnce();
                      rate.sleep();
                     }

              }

               while(local_x < temp_goal_x && boxID->obj[0].obj_type.c_str() != pole)
               {
                 pose.header.stamp = ros::Time::now();                          
                 pose.header.frame_id = "map";
                 pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yawt);
                 pose.pose.position.x = temp_goal_x; 
                 pose.pose.position.y = temp_goal_y;                      
                 local_setpos_pub.publish(pose);
                 ROS_INFO_ONCE("Poles passed. Heading to temporary target");
                  ros::spinOnce();
                  rate.sleep();
               }
            } 
            
	    old_pathfinder_node.object_list.erase(old_pathfinder_node.object_list.begin()); 
	    //old_pathfinder_node.object_list.resize(old_pathfinder_node.object_list.size() - 1);
	    obj_detected = false;
	      
        
    }else if(obj_detected && boxID->obj[0].obj_type.c_str() == pipe)
    {

        //******************************************************************************    
        //            MISSION PIPE
        //
        //******************************************************************************
           ROS_INFO_ONCE("MISSION SEGMENT :%s ",boxID->obj[0].obj_type.c_str());                 
                  
            float COEFx;
            float pipe_length = 12.0;  //estimated pipe length

            increm = 1;
            target_y =  boxID->obj[0].y_cnt - (pipe_length + OBJ_CLR);
          while(local_y >= target_y && increm >= 0)
          {


            while(abs(boxID->obj[0].x_cnt - local_x) > TOLRNCE ||
                  abs(boxID->obj[0].z_cnt - local_z) > TOLRNCE){
              
              //roll or drift to  get to the target
               pose.header.stamp = ros::Time::now();                  
               pose.header.frame_id = "map";
               pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,(-0.5*PI + yawt));                   
               pose.pose.position.x = boxID->obj[0].x_cnt;//increm;
               pose.pose.position.z = boxID->obj[0].z_cnt;//increm;                   
               local_setpos_pub.publish(pose);
               ROS_INFO_ONCE("Roll to head to:%s",boxID->obj[0].obj_type.c_str());
               //ROS_INFO("local y:%f, local_z:%f",local_y, local_z);
                ros::spinOnce();
                rate.sleep();                      
            }
               
            //advance forward to the target
                if(local_y > boxID->obj[0].y_cnt + OBJ_CLR)
                {
                  chk_target_y = 1.0;
                }else
                {
                  chk_target_y = -1.0;
                }      

            chk_target_y = (boxID->obj[0].y_cnt + (chk_target_y*OBJ_CLR - (1-increm)*pipe_length));              
            while(local_y > chk_target_y)                 
            {
              pose.header.stamp = ros::Time::now();                  
              pose.header.frame_id = "map";
               pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,(-0.5*PI + yawt));
               pose.pose.position.x = boxID->obj[0].x_cnt;
               pose.pose.position.y = chk_target_y;               
               pose.pose.position.z = boxID->obj[0].z_cnt;                   
              ROS_INFO_ONCE("Adance towards the:%s",boxID->obj[0].obj_type.c_str());
              //ROS_INFO("target_x:%f local y:%f, local_x:%f",boxID->obj[0].x_cnt, local_y, local_x);
              local_setpos_pub.publish(pose);
               ros::spinOnce();
               rate.sleep();
              
            }

            increm--;
          }
           
       
      old_pathfinder_node.object_list.erase(old_pathfinder_node.object_list.begin()); 
     // old_pathfinder_node.object_list.resize(old_pathfinder_node.object_list.size() - 1);
      obj_detected = false;

    }else if(obj_detected && boxID->obj[0].obj_type == tree)
    {
        //******************************************************************************    
        //            MISSION TREE
        //
        //******************************************************************************
             ROS_INFO_ONCE("MISSION SEGMENT :%s ",boxID->obj[0].obj_type.c_str()); 
            
      
            float t_x;   //x-axis target distance between consequetive obstacles
             float t_y;   //y-axis target distance between consequetive obstacle
            int W_pathID = 0;   //wide path id
             int N_pathID = 0;    //narrow path id
            float W_min_d = 1000.0;  //initialize guess of minimum distance to temp_goal for wide path
            float N_min_d = 1000.0;  //initialize minimum distance to temp_goal for narrow path

            float temp_goal_x = 11.5; //temporary goal point
            float temp_goal_y = -10.0; //temporary goal point

            float RL_ymin = -5.0;   
            float LL_ymax = -15.0;    
            int a, b;
               
            float temp_target_d;     //distance between a target point and the temp_goal point

            
            while(ros::ok() && local_x >= temp_goal_x)
            {

              while(obj_detected && boxID->obj[0].obj_type.c_str() == tree)
              {
                           

               for(int i = 0; i < boxNM; ++i)
                { 
                  //BOUNDERY CONDITIONS: 
                  //=============================================================                                      
                                            if(i == 0)
                                            {
                                              a = 0;
                                              b = 1;
                                            }else
                                            {
                                              a = 1;
                                              b = 0;
                                            }    
                  //=============================================================

                  if(abs(boxID->obj[i].y_min - (a*boxID->obj[i-1*a].y_max + b*LL_ymax)) >= UAV_WIDTH)
                  {                  
                    t_x = min(boxID->obj[i].x_cnt, (a*boxID->obj[i-1*a].x_cnt + b*local_x));
                    t_y = 0.5*(boxID->obj[i].y_min + (a*boxID->obj[i-1*a].y_max + b*LL_ymax));
                    temp_target_d = sqrt(pow((temp_goal_x - t_x),2) + pow((temp_goal_y - t_y),2));

                    if(W_min_d > temp_target_d)
                    {
                      W_min_d = temp_target_d;  //minimum distance to temp_goal through wide path
                      W_pathID = i;       //location of minimum distance to temp_goal through wide path             
                    }

                   // ROS_INFO("path obtained at W_pathID:%i",W_pathID);

                  }else if(abs(boxID->obj[i].y_min - (a*boxID->obj[i-1*a].y_max + b*LL_ymax)) < UAV_WIDTH &&
                    abs(boxID->obj[i].x_cnt - (a*boxID->obj[i-1*a].x_cnt + b*local_x)) >= DEPTH_DIF)
                  {
                    if(boxID->obj[i].x_cnt > (a*boxID->obj[i-1*a].x_cnt + b*local_x) && //flase left, true right
                      abs(boxID->obj[i].y_min - (a*boxID->obj[i-2*a].y_max + b*LL_ymax)) >= 2.0*MN_SOBT_D)
                    {
                      t_y = boxID->obj[i].y_min - MN_SOBT_D;
                      t_x = (a*boxID->obj[i-1*a].x_cnt + b*local_x) + MN_FOBT_D;

                    }else if(boxID->obj[i].x_cnt < (a*boxID->obj[i-1*a].x_cnt + b*local_x) &&
                      abs((a*boxID->obj[i-1*a].y_max + b*LL_ymax) - (b*boxID->obj[i+1*b].y_min + a*RL_ymin)) >= 2.0*MN_SOBT_D)
                    {
                      t_y = (a*boxID->obj[i-1*a].y_max + b*LL_ymax) + MN_SOBT_D;
                      t_x = boxID->obj[i].x_cnt + MN_FOBT_D;
                    }
                  temp_target_d = sqrt(pow((temp_goal_x - t_x),2) + pow((temp_goal_y - t_y),2));
                //find the minimum path id
                    if(N_min_d > temp_target_d)
                    {
                       N_min_d = temp_target_d;
                       N_pathID = i;                    
                    }
                    //ROS_INFO("path obtained at N_pathID:%i",N_pathID);
                  }else if(abs(boxID->obj[i].y_min - (a*boxID->obj[i-1*a].y_max + b*LL_ymax)) < UAV_WIDTH &&
                    abs(boxID->obj[i].x_cnt - (a*boxID->obj[i-1*a].x_cnt + b*local_x)) < DEPTH_DIF)
                  {
                    ROS_INFO_ONCE("THERE IS NO WAY TO PASS. ATEMPT TO ROLL");
                  }
                  

                }

                //START NAVIGATING BASED ON THE ABOVE OPTIONS

               if(abs(boxID->obj[W_pathID].y_min - (a*boxID->obj[W_pathID-1*a].y_max + b*LL_ymax)) >= UAV_WIDTH)
                  {               
                    target_x = min(boxID->obj[W_pathID].x_cnt, (a*boxID->obj[W_pathID-1*a].x_cnt + b*local_x)) - OBJ_CLR;                 
                    target_y = 0.5*(boxID->obj[W_pathID].y_min + (a*boxID->obj[W_pathID-1*a].y_max + b*LL_ymax));
                    ROS_INFO_ONCE("W_pathid:%i,target_x:%f, target_y:%f",W_pathID,target_x, target_y);
                    
                        
                      while(abs(target_y - local_y) > TOLRNCE){                                               
                        //coefy = abs(target_y - local_y);
                        pose.header.stamp = ros::Time::now();                        
                        pose.header.frame_id = "map";
                       pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-PI+yawt);
                       pose.pose.position.x = pose.pose.position.x;                   
                       pose.pose.position.y =  target_y;// + coefy*count; 
                       pose.pose.position.z = pose.pose.position.z;                                         
                       local_setpos_pub.publish(pose);
                       //ROS_INFO("Roll at local x:%f, local_y:%f",local_x, local_y);                       
                       ros::spinOnce();
                       rate.sleep();
                       
                       }                  
             
                       while(local_x >= target_x){
                         pose.header.stamp = ros::Time::now();                          
                         pose.header.frame_id = "map";
                         pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-PI+yawt);
                         pose.pose.position.x = target_x; 
                         pose.pose.position.y = pose.pose.position.y;
                         pose.pose.position.z = pose.pose.position.z;                      
                         //ROS_INFO("Adancing: local x:%f, local_y:%f",local_x, local_y);
                         local_setpos_pub.publish(pose);
                         ros::spinOnce();
                         rate.sleep();                       
                       }

                  }else if(abs(boxID->obj[N_pathID].y_min - (a*boxID->obj[N_pathID-1*a].y_max + b*LL_ymax)) < UAV_WIDTH &&
                    abs(boxID->obj[N_pathID].x_cnt - (a*boxID->obj[N_pathID-1*a].x_cnt + b*local_x)) >= DEPTH_DIF)
                  {
                    if(boxID->obj[N_pathID].x_cnt > (a*boxID->obj[N_pathID-1*a].x_cnt + b*local_x) &&
                      abs(boxID->obj[N_pathID].y_min - (a*boxID->obj[N_pathID-2*a].y_max + b*LL_ymax)) >= 2.0*MN_SOBT_D)
                    {
                      target_y = boxID->obj[N_pathID].y_min - MN_SOBT_D;
                      target_x = (a*boxID->obj[N_pathID-1*a].x_cnt + b*local_x) + MN_FOBT_D;
                      ROS_INFO_ONCE("LN_pathid:%i, tx:%f, ty:%f",N_pathID, target_x, target_y);
                    }else if(boxID->obj[N_pathID].x_cnt < (a*boxID->obj[N_pathID-1*a].x_cnt + b*local_x) &&
                      abs((a*boxID->obj[N_pathID-1*a].y_max + b*LL_ymax) - (b*boxID->obj[N_pathID+1*b].y_min + a*RL_ymin)) >= 2.0*MN_SOBT_D)
                    {
                      target_y = (a*boxID->obj[N_pathID-1*a].y_max + b*LL_ymax) + MN_SOBT_D;
                      target_x = boxID->obj[N_pathID].x_cnt + MN_FOBT_D;
                      //ROS_INFO("RN_pathid:%i, tx:%f, ty:%f",N_pathID, target_x, target_y);
                    }
                       
                      while(abs(target_y - local_y) > TOLRNCE){                                               
                        //coefy = abs(target_y - local_y);
                        pose.header.stamp = ros::Time::now();                        
                        pose.header.frame_id = "map";
                       pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-PI+yawt);
                       pose.pose.position.x = pose.pose.position.x;
                       pose.pose.position.y =  target_y;// + coefy*count; 
                       pose.pose.position.z = pose.pose.position.z;                                         
                       local_setpos_pub.publish(pose);
                       ROS_INFO_ONCE("Right pass through narrow path");                     
                       ros::spinOnce();
                       rate.sleep();                     
                       }                  
                                
                       while(local_x >= target_x){
                         pose.header.stamp = ros::Time::now();                          
                         pose.header.frame_id = "map";
                         pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-PI+yawt);
                         pose.pose.position.x = target_x; //COEF*count;
                         pose.pose.position.y = pose.pose.position.y;  
                         pose.pose.position.z = pose.pose.position.z;                     
                         local_setpos_pub.publish(pose);
                         ROS_INFO_ONCE("Adancing through Trees");
                         ros::spinOnce();
                         rate.sleep();                       
                       }            
                  }

                  //hover for a while
                  for(int i = 0; i < 100; ++i)
                     {
                      pose.header.stamp = ros::Time::now();                  
                      pose.header.frame_id = "map";
                      local_setpos_pub.publish(pose);
                      ROS_INFO_ONCE("HOVERING");
                      ros::spinOnce();
                      rate.sleep();
                     }

              }

               while(local_x > temp_goal_x && boxID->obj[0].obj_type.c_str() != tree)
               {
                 pose.header.stamp = ros::Time::now();                          
                 pose.header.frame_id = "map";
                 pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-PI+yawt);
                 pose.pose.position.x = temp_goal_x; 
                 pose.pose.position.y = temp_goal_y;
                 pose.pose.position.z = pose.pose.position.z;                        
                 local_setpos_pub.publish(pose);
                 ROS_INFO_ONCE("Trees passed. Heading to temporary target");
                  ros::spinOnce();
                  rate.sleep();
               }
            } 
            
            old_pathfinder_node.object_list.erase(old_pathfinder_node.object_list.begin()); 
            //old_pathfinder_node.object_list.resize(old_pathfinder_node.object_list.size() - 1);
            obj_detected = false;

    }else if(obj_detected && boxID->obj[0].obj_type.c_str() == net)
    {     
        //******************************************************************************    
        //            MISSION NET
        //
        //******************************************************************************
        ROS_INFO_ONCE("MISSION SEGMENT :%s ",boxID->obj[0].obj_type.c_str());
         
            
        float temp_target_x = 0.5*(boxID->obj[1].x_cnt + boxID->obj[0].x_cnt);
        target_x = min(boxID->obj[1].x_cnt , boxID->obj[0].x_cnt) - OBJ_CLR;

       // increm = 1;
        //count = 0;
        while(ros::ok() && local_x > target_x)
        {

            for (int i = 0; i < boxNM; ++i)
            {
              if(boxID->obj[i].z_cnt >= 0.5*ALT_LIMIT)
                {
                  target_z = 0.5*boxID->obj[i].z_min;
                  ROS_INFO_ONCE("NET UNDERPASS local_x=%f, target_z=%f",local_x, target_z);
                }else if(boxID->obj[i].z_cnt < 0.5*ALT_LIMIT)
                {
                 float z_max = 2.0*boxID->obj[i].z_cnt - boxID->obj[i].z_min;
                 target_z = 0.5*(ALT_LIMIT + z_max);
                 ROS_INFO_ONCE("NET OVERPASS local_x=%f, target_z=%f",local_x, target_z); 
                }
                        

                while(abs(local_z - target_z) > 0.2)
                {
                   pose.header.stamp = ros::Time::now();
                   pose.header.frame_id = "map";
                   pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-PI+yawt);
                   pose.pose.position.x = pose.pose.position.x;
                   pose.pose.position.y = pose.pose.position.y;
                   pose.pose.position.z = target_z;                      
                   local_setpos_pub.publish(pose);
                                  
                   ros::spinOnce();
                   rate.sleep();

                }
              
                while(local_x >= temp_target_x && i == 0)
                {
                    pose.header.stamp = ros::Time::now();
                    pose.header.frame_id = "map";
                    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-PI+yawt);
                    pose.pose.position.x = temp_target_x;
                    pose.pose.position.y = pose.pose.position.y;
                    pose.pose.position.z = target_z;                       
                    local_setpos_pub.publish(pose);                
                   // ROS_INFO("temp_target_x = %f, target_z=%f",temp_target_x, target_z);
                    ros::spinOnce();
                    rate.sleep();
                                  
                }

                while(local_x < temp_target_x && local_x >= target_x && i == 1)
                {
                    pose.header.stamp = ros::Time::now();
                    pose.header.frame_id = "map";
                    pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-PI+yawt);
                    pose.pose.position.x = target_x;
                    pose.pose.position.y = pose.pose.position.y;
                    pose.pose.position.z = target_z;                       
                    local_setpos_pub.publish(pose);                
                   // ROS_INFO("temp_target_x = %f, target_z=%f",temp_target_x, target_z);
                    ros::spinOnce();
                    rate.sleep();
                                  
                }
            }            
          
        } 
      
      
    old_pathfinder_node.object_list.erase(old_pathfinder_node.object_list.begin()); 
       // old_pathfinder_node.object_list.resize(old_pathfinder_node.object_list.size() - 1);
      obj_detected = false;

    }else if(obj_detected && boxID->obj[0].obj_type == land)
    { 

        //******************************************************************************    
        //            MISSION LAND
        //
        //******************************************************************************
             ROS_INFO_ONCE("MISSION SEGMENT :%s ",boxID->obj[0].obj_type.c_str());
            
            
                          
             while(abs(boxID->obj[0].x_cnt - local_x) > TOLRNCE ||
                        abs(boxID->obj[0].y_cnt - local_y) > TOLRNCE){
               pose.header.stamp = ros::Time::now();                          
               pose.header.frame_id = "map";
               pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,(-PI + yawt));
               pose.pose.position.x = boxID->obj[0].x_cnt;               
               pose.pose.position.y = boxID->obj[0].y_cnt;
               pose.pose.position.z = pose.pose.position.z;                       
               local_setpos_pub.publish(pose);
               ros::spinOnce();
               rate.sleep();               
             } 

               
             while(local_z >= boxID->obj[0].z_cnt){
               pose.header.stamp = ros::Time::now();                          
               pose.header.frame_id = "map";
               pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,(-PI + yawt));
               pose.pose.position.x = pose.pose.position.x;               
               pose.pose.position.y = pose.pose.position.y;
               pose.pose.position.z = boxID->obj[0].z_cnt;                       
               local_setpos_pub.publish(pose);
               ROS_INFO_ONCE("HEADING TO LANDING SITE ...");
               ros::spinOnce();
               rate.sleep();               
             } 
             
             ///
           // while(obj_detected && !(old_pathfinder_node.land_client.call(land_cmd) && 
            //    land_cmd.response.success)){
            //    ros::spinOnce;
             //   rate.sleep();
            //}
            ///

                //obj_detected = false;
            //old_pathfinder_node.object_list.erase(old_pathfinder_node.object_list.begin()); 
          //old_pathfinder_node.object_list.resize(old_pathfinder_node.object_list.size() - 1);
            break;
    }               


        //}else{
    //===================================================================================================

    //              YAW-SEARCH FOR THE KNOWN OBJECTS

    //===================================================================================================

   if(!obj_detected && old_pathfinder_node.object_list.size() > 1)
    {
        count = 0;
        while(!obj_detected && rot_l <= ROT_MAX){ 
            rot_l = count*PI/360.0;             
            pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,rot_l);
            
            local_setpos_pub.publish(pose);
            
            ROS_INFO("Yaw left by angle:%f, count:%i",rot_l, count);

            for(int i = 0; i < old_pathfinder_node.object_list.size(); ++i)
            {
              if(old_pathfinder_node.object_list[i] == boxID->obj[0].obj_type && i == 0)              {
                
                obj_detected = true;
              }
            }
            ros::spinOnce();
            rate.sleep();
            count++;
        }               
        
            count = 0;
            rot_r = rot_l;
        while(!obj_detected && rot_r >= ROT_MIN){
            rot_r = rot_l - count*PI/360.0; 
            pose.header.stamp = ros::Time::now();                
            pose.header.frame_id = "map";
            pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,rot_r);
            
            local_setpos_pub.publish(pose);             
            ROS_INFO("Yaw right by angle:%f, count:%i",rot_r, count);

            for(int i = 0; i < old_pathfinder_node.object_list.size(); ++i)
            {
              if(old_pathfinder_node.object_list[i] == boxID->obj[0].obj_type && i == 0)
              {
               
                obj_detected = true;
              }
            }

            ros::spinOnce();
            rate.sleep();
            count++;               
        }

    
    }else if(!obj_detected && old_pathfinder_node.object_list.size() == 1)
    {  

    //===================================================================================================

    //              ROLL-SEARCH FOR THE KNOWN OBJECTS
    
    //===================================================================================================
      while(!obj_detected && abs(local_y) > (5.0 + MN_SOBT_D) && abs(local_y) < (15.0 - MN_SOBT_D))
          {
              while(!obj_detected && local_y < 0.5*(-5.0 - MN_SOBT_D))
              {
                 pose.header.stamp = ros::Time::now();                   
               pose.header.frame_id = "map";
               pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yawt);                   
               pose.pose.position.y =  0.5*(-5.0 - MN_SOBT_D);                                  
               local_setpos_pub.publish(pose);
               ROS_INFO("Roll to left first");
               for(int i = 0; i < old_pathfinder_node.object_list.size(); ++i)
                {
                  if(old_pathfinder_node.object_list[i] == boxID->obj[0].obj_type && i == 0)
                  {
                   
                    obj_detected = true;
                  }
                }
                ros::spinOnce();
                  rate.sleep();
            }

            while(!obj_detected && local_y > -10.0)
              {
                 pose.header.stamp = ros::Time::now();                   
               pose.header.frame_id = "map";
               pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yawt);                   
               pose.pose.position.y =  -10.0;                                  
               local_setpos_pub.publish(pose);
               ROS_INFO("Roll to left first");
               for(int i = 0; i < old_pathfinder_node.object_list.size(); ++i)
                {
                  if(old_pathfinder_node.object_list[i] == boxID->obj[0].obj_type && i == 0)
                  {
                   
                    obj_detected = true;
                  }
                }
                ros::spinOnce();
                  rate.sleep();
            }

            while(!obj_detected && local_y > 0.5*(-15.0 + MN_SOBT_D))
              {
                 pose.header.stamp = ros::Time::now();                   
               pose.header.frame_id = "map";
               pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yawt);                   
               pose.pose.position.y =  0.5*(-15.0 + MN_SOBT_D);                                  
               local_setpos_pub.publish(pose);
               ROS_INFO("Roll to right second");
               for(int i = 0; i < old_pathfinder_node.object_list.size(); ++i)
                {
                  if(old_pathfinder_node.object_list[i] == boxID->obj[0].obj_type && i == 0)
                  {
                   
                    obj_detected = true;
                  }
                }
                ros::spinOnce();
                  rate.sleep();
            }

            while(!obj_detected && local_y < -10.0)
              {
                 pose.header.stamp = ros::Time::now();                   
               pose.header.frame_id = "map";
               pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,yawt);                   
               pose.pose.position.y =  -10.0;                                  
               local_setpos_pub.publish(pose);
               ROS_INFO("Roll to left first");
               for(auto i = old_pathfinder_node.object_list.begin(); 
                    i != old_pathfinder_node.object_list.end(); i++){                                  
                    if(*i == boxID->obj[0].obj_type && old_pathfinder_node.object_list[0] == *i){                       
                        ROS_INFO("SEARCHING FOR LANDING PAD...");

                        obj_detected = true;
                    }                        
                }
                ros::spinOnce();
                  rate.sleep();
            }

           
          }

    }

        
}
                                 
                      
            
            

        
        
    /*    
        count = 0;
        while(local_y <= 0.0 && local_x >= 6){
               pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
               //pose.pose.position.x = pose.pose.position.x + COEF*count;
               pose.pose.position.y = 1.0;
                pose.pose.position.z = old_pathfinder_node.z_cnt;                   
               ROS_INFO("Passthrough poles:%f",pose.pose.position.x);
               local_setpos_pub.publish(pose);
               ros::spinOnce();
               rate.sleep();
               count++;
        }

        count = 0;
        while(local_x <= 23.0){
               pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
               pose.pose.position.x = pose.pose.position.x + COEF*count;
               pose.pose.position.y = old_pathfinder_node.y_cnt;
                pose.pose.position.z = old_pathfinder_node.z_cnt;                   
               ROS_INFO("Second phase pitch advance:%f",pose.pose.position.x);
               local_setpos_pub.publish(pose);
               ros::spinOnce();
               rate.sleep();
               count++;
        }
              
              
         count = 0;
            rot_r = 0.0;                
            while(obj_detected && rot_r >= ROT_MIN){
                rot_r = - count*PI/360.0; 
                pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,rot_r);
                
                local_setpos_pub.publish(pose);             
                ROS_INFO("detected at yaw_ang_r:%f, count:%i",rot_r, count);                
                ros::spinOnce();
                rate.sleep();
                count++;               
            //}
                //target angle fuse with (ende's/wonde's)
                //int obtained = string::compare(string s1, string s2);
                string window;
                //string window;              
                if(window.compare(window) == 0 && rot_r <= -0.9877559){                  
                    obj_detected = false;
                    //Let, x_cntance to the center of the object(window) is detected at = 4.0
                    float target_x = obj_x_cnt*cos(rot_r) + pose.pose.position.x;
                    count = 0;
                  while(local_x < target_x){
                   pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,-PI/2.0);                   
                   pose.pose.position.x =  pose.pose.position.x + COEF*count;
                   pose.pose.position.z = old_pathfinder_node.z_cnt;
                   //pose.pose.position.z = pose.pose.position.z;                                      
                   
                    local_setpos_pub.publish(pose);
                    ROS_INFO("Second phase Roll advance:%f",pose.pose.position.x);
                    ros::spinOnce();
                    rate.sleep();
                    count++;
                   }

                   count = 0;
                   float target_y = pose.pose.position.y + obj_x_cnt*sin(rot_r);
                   while(abs(local_y) <= abs(target_y) + 12.50){
                   pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
                   pose.pose.position.y = pose.pose.position.y + COEF*count;
                   //pose.pose.position.y = old_pathfinder_node.y_cnt;
                    pose.pose.position.z = old_pathfinder_node.z_cnt;                   
                   ROS_INFO("Pitch advance:%f",pose.pose.position.x);
                   local_setpos_pub.publish(pose);
                   ros::spinOnce();
                   rate.sleep();
                   count++;
                   }
                }
                 
            }*/    
           /*
                    // yaw (z-axis rotation)
                    float qw = pose.pose.orientation.w;
                    float qx = pose.pose.orientation.x;
                    float qy = pose.pose.orientation.y;
                    float qz = pose.pose.orientation.z;
                    double siny_cosp = +2.0 * (qw * qz + qx * qy);
                    double cosy_cosp = +1.0 - 2.0 * (qy * qy + qz * qz);  
                    float yaw = atan2(siny_cosp, cosy_cosp);
                    ROS_INFO("yaw:%f",yaw);
                    */  
            





        
    /*

        ROS_INFO_ONCE("LANDING AT land");
        while(!(old_pathfinder_node.land_client.call(land_cmd) && land_cmd.response.success)){
            ros::spinOnce;
            rate.sleep();
        } 
         */       
            
        
    
    


}