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


void BoundBox_info_cb(const object_localization::detected_object::ConstPtr& msg)
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
      const object_localization::detected_object_info &data = msg->info[i];
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
  const float ALT_LIMIT = 2.0;
    float yawt;
    int count; 
    int increm;   
  const float PI = 3.14159265359;
  const float TOLRNCE = 0.2;       //acceptance radius
  const float OBJ_CLR = 1.0;    //object clearance to avoid colission
  const float ALT_SAFETY = 0.65;     //maintain safe altitude to avoid roof collision
  //const float ROLL_SAFETY = 0.75;
    bool obj_detected = false;
  const float MARGIN = 0.3;
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
  
  
  ros::Subscriber  BoundBox_sub = nh.subscribe<object_localization::detected_object> 
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

//hover for a while
       for(int i = 0; i < 5000; ++i)
       {
        pose.header.stamp = ros::Time::now();                  
        pose.header.frame_id = "map";
        local_setpos_pub.publish(pose);       
        ros::spinOnce();
        rate.sleep();
       }

 ROS_INFO("LANDING...");
      while (!(land_client.call(land_cmd) && land_cmd.response.success)) {                    
          pose.header.stamp = ros::Time::now();                  
          pose.header.frame_id = "map";
          local_setpos_pub.publish(pose);
          ros::spinOnce();
          rate.sleep();
      } 
    

}
