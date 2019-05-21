// zed stereo camera subscription code here


// include
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <zed_data_subscribe/detected_object.h>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf/transform_datatypes.h>

#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <cstddef>
#include <math.h>
#include <vector>
#include <algorithm>

//

#define RAD2DEG 57.295779513
using namespace std;

//string bado= "(null)";
/**
 * Subscriber callbacks
 */

int u;
int v;
double tx ;
double ty ;
double tz ;
double roll, pitch, yaw;
int n=0;

float X, Y, Z;

struct objects_boundbox
{
string name;
float  u_c;
float  v_c;
float  u_min;
float  u_max;
float  v_min;
float  v_max;
};

struct objects_bbox
{
vector<objects_boundbox> obj;
}obx;

struct objects_
{
string name;
float  Y_min;
float  X;
float  Y;
float  Z;
float  distance;
};

struct objects
{
vector<objects_> object;
}ob;


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
    ROS_INFO("Center distance : %g m", depths[centerIdx]);
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
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

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

     int arrayPosition1;
     int arrayPosXmin  ; // X has an offset of 0
     int arrayPosYmin  ; // Y has an offset of 4
     int arrayPosZmin  ; // Z has an offset of 8
     float dis, xmin, ymin, zmin;
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

      arrayPosition1 = (obb->obj[i].v_c)*pointCL.row_step + (obb->obj[i].u_max)*pointCL.point_step;
      arrayPosXmin     = arrayPosition1 + pointCL.fields[0].offset; // X has an offset of 0
      arrayPosYmin     = arrayPosition1 + pointCL.fields[1].offset; // Y has an offset of 4
      arrayPosZmin     = arrayPosition1 + pointCL.fields[2].offset; // Z has an offset of 8

      memcpy(&xmin, &pointCL.data[arrayPosXmin], sizeof(float));
      memcpy(&ymin, &pointCL.data[arrayPosYmin], sizeof(float));
      memcpy(&zmin, &pointCL.data[arrayPosZmin], sizeof(float)); 

      ob.object.push_back({obb->obj[i].name.c_str(),ymin,X,Y,Z,dis});
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
 n = msg->data;
 ROS_INFO("number of detected objects: %d",msg->data);
}
 
int main(int argc, char **argv)
{


  ros::init(argc, argv, "zed_data_sub");


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

 ros::Publisher detected_obj_pub = n.advertise<zed_data_subscribe::detected_object>("/detected_object/info",10);
 zed_data_subscribe::detected_object  detected_obj;
 zed_data_subscribe::detected_object_info  detected_obj_info;

 ros::Rate rate(70.0);  
 geometry_msgs::PoseStamped loc_pos;
 ros::Time last_request = ros::Time::now();
 float ox,oy,oz;

    // sort(obb.obj.begin(),obb.obj.end(),[](objects_bbox a, objects_bbox b)->bool {return a.obj[i].u_c < b.obj[i].u_c;});
  objects* obb = &ob;
     // ROS_INFO("n = %d",obb->object.size());

while(ros::ok() )
{
 loc_pos.header.stamp = ros::Time::now();
 loc_pos.header.frame_id ="map";
 loc_pos.pose.position.x = tx;
 loc_pos.pose.position.y = ty;
 loc_pos.pose.position.z = tz;

 loc_pos.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
 ox = tx + X*cos(yaw)-Y*sin(yaw);
 oy = ty + Y*cos(yaw) + X*sin(yaw);
 oz = tz + Z;

  for (int i=0; i<obb->object.size();++i )
 {
    detected_obj_info.name      = obb->object[i].name.c_str();
    detected_obj_info.Y_min     = obb->object[i].Y_min;
    detected_obj_info.X         = obb->object[i].X;
    detected_obj_info.Y         = obb->object[i].Y;
    detected_obj_info.Z         = obb->object[i].Z;
    detected_obj_info.distance  = obb->object[i].distance;
    detected_obj.info.push_back(detected_obj_info);

    ROS_INFO("%s  XYZ_map @ (%f, %f, %f), Y_min = %f , Distance = %f  ",obb->object[i].name.c_str(),
    obb->object[i].X,obb->object[i].Y,obb->object[i].Z,obb->object[i].Y_min,obb->object[i].distance);
 }

 detected_obj_pub.publish(detected_obj);
 detected_obj.info.clear();
 // ROS_INFO("X");
 // ROS_INFO("Camera position wrt map: (%f,%f,%f)",tx,ty,tz);
 // ROS_INFO("Camera rotation wrt camera: (%f,%f,%f)",roll*RAD2DEG,pitch*RAD2DEG,yaw*RAD2DEG);
 // ROS_INFO("Object position wrt map: (%f,%f,%f)",ox,oy,oz);
 local_pos_pub.publish(loc_pos);
 // ros::spin();
 ros::spinOnce();
 rate.sleep();

}


  return 0;
}

