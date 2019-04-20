// zed stereo camera subscription code here

// include
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <string>


/**
 * Subscriber callbacks
 */
void camera_depth_callback(const sensor_msgs::Image::ConstPtr& msg)
{
    // Get a pointer to the depth values casting the data
    // pointer to floating point
    float* depths = (float*)(&msg->data[0]);

    // Image coordinates of the center pixel
    int u = msg->width / 2;
    int v = msg->height / 2;

    // Linear index of the center pixel
    int centerIdx = u + msg->width * v;

    // Output the measure
    ROS_INFO("Center distance : %g m", depths[centerIdx]);
}


int main(int argc, char **argv)
{


  ros::init(argc, argv, "zed_data_sub");


  ros::NodeHandle n;


  ros::Subscriber sub        = n.subscribe("/zed/depth/depth_registered", 10, camera_depth_callback);

  ros::spin();


  return 0;
}

