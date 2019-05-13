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

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf/transform_datatypes.h>

#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <cstddef>


#define RAD2DEG 57.295779513
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

void boundingbox_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr&  msg)
{

   // int  i = 0;
   // int nn = n;
  for (int i =0; i<msg->bounding_boxes.size();++i)
   {

      const darknet_ros_msgs::BoundingBox &data = msg->bounding_boxes[i];
      float  X_c = (data.xmin + data.xmax )/2;
      float  Y_c = (data.ymin + data.ymax )/2;
      ROS_INFO("%s center is @ (%f,%f) ",data.Class.c_str(),X_c,Y_c );     
     
   
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
    ROS_INFO("Received pose in '%s' frame : X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.2f Y: %.2f",
             msg->header.frame_id.c_str(),
             tx, ty, tz,
             roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
}

void imageRightRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // ROS_INFO("Right Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
}

void imageLeftRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg) {
    // ROS_INFO("Left Rectified image received from ZED - Size: %dx%d", msg->width, msg->height);
}

void pixelTo3DXYZ_callback(const sensor_msgs::PointCloud2 pointCL)
{


    int arrayPosition = (v-300)*pointCL.row_step + (u-400)*pointCL.point_step;
    int arrayPosX     = arrayPosition + pointCL.fields[0].offset; // X has an offset of 0
    int arrayPosY     = arrayPosition + pointCL.fields[1].offset; // Y has an offset of 4
    int arrayPosZ     = arrayPosition + pointCL.fields[2].offset; // Z has an offset of 8
 
    float X ;
    float Y ;
    float Z ;
 
    memcpy(&X, &pointCL.data[arrayPosX], sizeof(float));
    memcpy(&Y, &pointCL.data[arrayPosY], sizeof(float));
    memcpy(&Z, &pointCL.data[arrayPosZ], sizeof(float));
   
    ROS_INFO("XYZ = (%f,%f,%f)",X,Y,Z);
 
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
  ros::Subscriber sub               = n.subscribe("/zed/depth/depth_registered", 10, camera_depth_callback);
  ros::Subscriber subpcl            = n.subscribe("/zed/point_cloud/cloud_registered", 10, pixelTo3DXYZ_callback);
  ros::Subscriber subcenter         = n.subscribe("/darknet_ros/bounding_boxes", 10, boundingbox_callback); // checking data subscription from objected detector package
  ros::Subscriber subOdom           = n.subscribe("/zed/odom", 10, odomCallback);
  ros::Subscriber subPose           = n.subscribe("/zed/pose", 10, poseCallback);
  ros::Subscriber subRightRectified = n.subscribe("/zed/right/image_rect_color", 10,imageRightRectifiedCallback);
  ros::Subscriber subLeftRectified  = n.subscribe("/zed/left/image_rect_color", 10,imageLeftRectifiedCallback);
  ros::Subscriber subObj_num        = n.subscribe("/darknet_ros/found_object",10, numOfdetectedObjetCallback);

  ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>
            ("/mavros/vision_pose/pose", 100); //mocap vision_pose /mavros/vision_pose/pose

 ros::Rate rate(70.0);  
 geometry_msgs::PoseStamped loc_pos;
 ros::Time last_request = ros::Time::now();

while(ros::ok() )
{
 loc_pos.header.stamp = ros::Time::now();
 loc_pos.header.frame_id ="map";
 loc_pos.pose.position.x = tx;
 loc_pos.pose.position.y = ty;
 loc_pos.pose.position.z = tz;

 loc_pos.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch,yaw);
 ROS_INFO("X");
 ROS_INFO("X: %f",loc_pos.pose.position.x);
 local_pos_pub.publish(loc_pos);
 // ros::spin();
 ros::spinOnce();
 rate.sleep();

}


  return 0;
}

