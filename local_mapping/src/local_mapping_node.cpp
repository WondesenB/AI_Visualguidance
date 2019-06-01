// zed stereo camera subscription code here

#include "local_mapping_node.h"

 



void boundingbox_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr&  msg)
{
   
 
  obx.obj.clear();
  for (int i =0; i<msg->bounding_boxes.size();++i)
   {

      const darknet_ros_msgs::BoundingBox &data = msg->bounding_boxes[i];
      float  X_c = (data.xmin + data.xmax )/2;
      float  Y_c = (data.ymin + data.ymax )/2;
      obx.obj.push_back({data.Class.c_str(),X_c,Y_c,data.xmin,data.xmax,data.ymin,data.ymax});
      
      // ROS_INFO("%s center is @ (%f,%f) ",data.Class.c_str(),X_c,Y_c);     
     
   
   }


}

void camera_depth_callback(const sensor_msgs::Image::ConstPtr& msg)
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
    ROS_INFO("Center distance : %g m", depths[centerIdx]);
    }
}

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {

    // Camera position in map frame
    double tx = msg->pose.pose.position.x;
    double ty = msg->pose.pose.position.y;
    double tz = msg->pose.pose.position.z;

    // Orientation quaternion
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Output the measure
    // ROS_INFO("Received odom in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
    //          msg->header.frame_id.c_str(),
    //          tx, ty, tz,
    //          roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {

   

    // Camera position in map frame
     tx = msg->pose.position.x;
     ty = msg->pose.position.y;
     tz = msg->pose.position.z;

    // Orientation quaternion
     rx = msg->pose.orientation.x;
     ry = msg->pose.orientation.y;
     rz = msg->pose.orientation.z;
     w  = msg->pose.orientation.w;

    tf2::Quaternion q(rx,ry,rz,w);

    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);

    // Roll Pitch and Yaw from rotation matrix
    
    m.getRPY(roll, pitch, yaw);

   // Output the measure
    // ROS_INFO("Received pose in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
    //          msg->header.frame_id.c_str(),
    //          tx, ty, tz,
    //          roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
}

void imageRightRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // ROS_INFO("Right Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
}

void imageLeftRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // ROS_INFO("Left Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
}

void pixelTo3DXYZ_callback(const sensor_msgs::PointCloud2 pointCL)
{

     
     objects_bbox* obb = &obx;
     // objects* obo = &ob;
     int arrayPosition;
     int arrayPosX  ; // X has an offset of 0
     int arrayPosY  ; // Y has an offset of 4
     int arrayPosZ  ; // Z has an offset of 8

     // Y_min
     int arrayPosition1;
     int arrayPosXmin  ; // X has an offset of 0
     int arrayPosYmin  ; // Y has an offset of 4
     int arrayPosZmin  ; // Z has an offset of 8
     float dis, Xymin, Yymin, Zymin;

     // Z_min
     int arrayPosition2;
     int arrayPosXZmin  ; // X has an offset of 0
     int arrayPosYZmin  ; // Y has an offset of 4
     int arrayPosZZmin  ; // Z has an offset of 8
     float Xzmin,Yzmin, Zzmin, area;
     ob.object.clear();

     for (int i =0; i<obb->obj.size();++i)
     {

      arrayPosition = (obb->obj[i].v_c)*pointCL.row_step + (obb->obj[i].u_c)*pointCL.point_step;
      arrayPosX     = arrayPosition + pointCL.fields[0].offset; // X has an offset of 0
      arrayPosY     = arrayPosition + pointCL.fields[1].offset; // Y has an offset of 4
      arrayPosZ     = arrayPosition + pointCL.fields[2].offset; // Z has an offset of 8
   
     
      memcpy(&X, &pointCL.data[arrayPosX], sizeof(float));
      memcpy(&Y, &pointCL.data[arrayPosY], sizeof(float));
      memcpy(&Z, &pointCL.data[arrayPosZ], sizeof(float)); 
      dis = sqrt(X*X+Y*Y+Z*Z);

     // Y_min
      arrayPosition1 = (obb->obj[i].v_c)*pointCL.row_step + (obb->obj[i].u_max)*pointCL.point_step;
      arrayPosXmin     = arrayPosition1 + pointCL.fields[0].offset; // X has an offset of 0
      arrayPosYmin     = arrayPosition1 + pointCL.fields[1].offset; // Y has an offset of 4
      arrayPosZmin     = arrayPosition1 + pointCL.fields[2].offset; // Z has an offset of 8

      memcpy(&Xymin, &pointCL.data[arrayPosXmin], sizeof(float));
      memcpy(&Yymin, &pointCL.data[arrayPosYmin], sizeof(float));
      memcpy(&Zymin, &pointCL.data[arrayPosZmin], sizeof(float)); 

     // Z_min
      arrayPosition2 = (obb->obj[i].v_max)*pointCL.row_step + (obb->obj[i].u_c)*pointCL.point_step;
      arrayPosXZmin     = arrayPosition2 + pointCL.fields[0].offset; // X has an offset of 0
      arrayPosYZmin     = arrayPosition2 + pointCL.fields[1].offset; // Y has an offset of 4
      arrayPosZZmin     = arrayPosition2 + pointCL.fields[2].offset; // Z has an offset of 8

      memcpy(&Xzmin, &pointCL.data[arrayPosXZmin], sizeof(float));
      memcpy(&Yzmin, &pointCL.data[arrayPosYZmin], sizeof(float));
      memcpy(&Zzmin, &pointCL.data[arrayPosZZmin], sizeof(float));
      area = (2.0f*abs(Y-Yymin))*(2.0f*abs(Z-Zzmin));

     if (std::isfinite(X) && std::isfinite(Y) && std::isfinite(Z) )
     {
      ob.object.push_back({obb->obj[i].name.c_str(),Yymin,Zzmin,X,Y,Z,dis,area});
     }
      // ROS_INFO("%s Y_min is @ (%f,%f)",obb->obj[i].name.c_str(),obb->obj[i].u_min,obb->obj[i].v_c);   
      // ROS_INFO("detected object info,name = %s , XYZ = (%f,%f,%f) , Y_min = %f, distance = %f ",
      //   ob.object[i].name.c_str(),ob.object[i].X,ob.object[i].Y,ob.object[i].Z,ob.object[i].Y_min,ob.object[i].distance);
     }

     obx.obj.clear();
     // sort(ob.begin(),ob.end(),[](objects a, objects b)->bool {return a.Y_min < b.Y_min;});

     // for (int i=0; i<ob.size();++i )
     // {
     //  ROS_INFO("%s  %f  %f  %f %f %f  %f",ob[i].name.c_str(),ob[i].Y_min,ob[i].Z_min,ob[i].X,ob[i].Y,ob[i].Z,ob[i].distance);
     // }
    
 
}

void numOfdetectedObjetCallback(const std_msgs::Int8::ConstPtr& msg)
{
 int n = msg->data;
 ROS_INFO("number of detected objects: %d",msg->data);
}
 
int main(int argc, char **argv)
{


  ros::init(argc, argv, "local_mapping_node");

  ros::NodeHandle n;
  ros::Subscriber sub               = n.subscribe("/zed/zed_node/depth/depth_registered", 10, camera_depth_callback);
  ros::Subscriber subpcl            = n.subscribe("/zed/zed_node/point_cloud/cloud_registered", 10, pixelTo3DXYZ_callback);
  ros::Subscriber subcenter         = n.subscribe("/darknet_ros/bounding_boxes", 10, boundingbox_callback); // checking data subscription from objected detector package
  // ros::Subscriber subOdom           = n.subscribe("/zed/zed_node/odom", 10, odomCallback);
  ros::Subscriber subPose           = n.subscribe("/zed/zed_node/pose", 10, poseCallback);
  // ros::Subscriber subRightRectified = n.subscribe("/zed/right/image_rect_color", 10,imageRightRectifiedCallback);
  // ros::Subscriber subLeftRectified  = n.subscribe("/zed/left/image_rect_color", 10,imageLeftRectifiedCallback);
  // ros::Subscriber subObj_num        = n.subscribe("/darknet_ros/found_object",10, numOfdetectedObjetCallback);

  ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>
            ("/mavros/vision_pose/pose", 100); //mocap vision_pose /mavros/vision_pose/pose

 ros::Publisher detected_obj_pub = n.advertise<local_mapping::detected_object>("/detected_object/info",10);
 local_mapping::detected_object  detected_obj;
 local_mapping::detected_object_info  detected_obj_info;

 
  static tf2_ros::TransformBroadcaster Qudpose_broadcaster;
  geometry_msgs::TransformStamped transformStamped;
 // tf::TransformBroadcaster zedbroadcaster;
  zed2dronebase_transform ();

 ros::Rate rate(70.0); 

 
 geometry_msgs::PoseStamped loc_pos;
 ros::Time last_request = ros::Time::now();
  // float ox,oy,oz;

    // sort(obb.obj.begin(),obb.obj.end(),[](objects_bbox a, objects_bbox b)->bool {return a.obj[i].u_c < b.obj[i].u_c;});
  objects* obb = &ob;
     // ROS_INFO("n = %d",obb->object.size());


  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);

  // geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PointStamped XYZ_in , Ymin_in, Zmin_in ;
  geometry_msgs::PointStamped XYZ_out,Ymin_out, Zmin_out ;

while(ros::ok() )
{

  // Tf2 tansform 
  transformStamped.header.stamp = ros::Time::now();
  transformStamped.header.frame_id = "map";
  transformStamped.child_frame_id = "base_drone";
  transformStamped.transform.translation.x = tx;
  transformStamped.transform.translation.y = ty;
  transformStamped.transform.translation.z = tz;
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  transformStamped.transform.rotation.x = quat.x();
  transformStamped.transform.rotation.y = quat.y();
  transformStamped.transform.rotation.z = quat.z();
  transformStamped.transform.rotation.w = quat.w();

  Qudpose_broadcaster.sendTransform(transformStamped);

    // zedbroadcaster.sendTransform(
    //   tf::StampedTransform(
    //     tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.5, 0.0, -0.2)),
    //     ros::Time::now(),"base_drone", "base_zed"));

 loc_pos.header.stamp = ros::Time::now();
 loc_pos.header.frame_id ="map";
 loc_pos.pose.position.x = tx;
 loc_pos.pose.position.y = ty;
 loc_pos.pose.position.z = tz;
 loc_pos.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);

 local_pos_pub.publish(loc_pos);

 //Transformation
 // ox = tx + X*cos(yaw)-Y*sin(yaw);
 // oy = ty + Y*cos(yaw) + X*sin(yaw);
 // oz = tz + Z;


//publisher
  for (int i=0; i<obb->object.size();++i )
 {

    XYZ_in.header.stamp = ros::Time();
    XYZ_in.point.x = obb->object[i].X;
    XYZ_in.point.y = obb->object[i].Y;
    XYZ_in.point.z = obb->object[i].Z;

    Ymin_in.header.stamp = ros::Time();
    Ymin_in.point.x = obb->object[i].X;
    Ymin_in.point.y = obb->object[i].Y_min;
    Ymin_in.point.z = obb->object[i].Z;

    Zmin_in.header.stamp = ros::Time();
    Zmin_in.point.x = obb->object[i].X;
    Zmin_in.point.y = obb->object[i].Y;
    Zmin_in.point.z = obb->object[i].Z_min;

  try {
      transformStamped = tfBuffer.lookupTransform("map", "camera_center",ros::Time(0),ros::Duration(10.0));
      tf2::doTransform(XYZ_in, XYZ_out, transformStamped);
      tf2::doTransform(Ymin_in, Ymin_out, transformStamped);
      tf2::doTransform(Zmin_in, Zmin_out, transformStamped);

      ROS_INFO("XYZ wrt camera = (%f, %f, %f), XYZ wrt map = (%f, %f, %f)" ,XYZ_in.point.x,XYZ_in.point.y,XYZ_in.point.y,XYZ_out.point.x, XYZ_out.point.y,XYZ_out.point.z );

      } 
  catch (tf2::TransformException &ex) 

      {

      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
      continue;
      }

    detected_obj_info.name      = obb->object[i].name.c_str();
    detected_obj_info.Y_min     = Ymin_out.point.y;
    detected_obj_info.Z_min     = Zmin_out.point.z;
    detected_obj_info.X         = XYZ_out.point.x;
    detected_obj_info.Y         = XYZ_out.point.y;
    detected_obj_info.Z         = XYZ_out.point.z;
    detected_obj_info.distance  = obb->object[i].distance;
    detected_obj_info.area      =obb->object[i].area;
    detected_obj.info.push_back(detected_obj_info);

    // ROS_INFO("%s  XYZ_map @ (%f, %f, %f), Y_min = %f , Z_min = %f , Distance = %f , Area = %f ",obb->object[i].name.c_str(),
    // obb->object[i].X,obb->object[i].Y,obb->object[i].Z,obb->object[i].Y_min,obb->object[i].Y_min,obb->object[i].distance,obb->object[i].area);
 }
 if(obb->object.size()<1)
  {
    detected_obj_info.name      = "null";
    detected_obj_info.Y_min     = 0.0;
    detected_obj_info.Z_min     = 0.0;
    detected_obj_info.X         = 0.0;
    detected_obj_info.Y         = 0.0;
    detected_obj_info.Z         = 0.0;
    detected_obj_info.distance  = 0.0;
    detected_obj_info.area      = 0.0;
    detected_obj.info.push_back(detected_obj_info);

  }
 detected_obj.header.stamp = ros::Time::now();
 detected_obj.header.frame_id ="map";
 detected_obj_pub.publish(detected_obj);
 detected_obj.info.clear();
 // ROS_INFO("X");
 // ROS_INFO("Camera position wrt map: (%f,%f,%f)",tx,ty,tz);
 // ROS_INFO("Camera rotation wrt camera: (%f,%f,%f)",roll*RAD2DEG,pitch*RAD2DEG,yaw*RAD2DEG);
 // ROS_INFO("Object position wrt map: (%f,%f,%f)",ox,oy,oz);

 // ros::spin();
 ros::spinOnce();
 rate.sleep();

}


  return 0;
}


// function definition

void zed2dronebase_transform (void)
{
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;

  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "base_drone";
  static_transformStamped.child_frame_id = "camera_center";
  static_transformStamped.transform.translation.x = 0.15;
  static_transformStamped.transform.translation.y = 0.0;
  static_transformStamped.transform.translation.z = -0.10;
  tf2::Quaternion quat;
  quat.setRPY(0.0, 0.0, 0.0);
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  static_broadcaster.sendTransform(static_transformStamped);
}
