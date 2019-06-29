// zed stereo camera subscription code here

#include "local_mapping_node.h"

 



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


 
int main(int argc, char **argv)
{


  ros::init(argc, argv, "local_mapping_node");

  ros::NodeHandle n;

  // ros::Subscriber subOdom           = n.subscribe("/zed/zed_node/odom", 10, odomCallback);
  ros::Subscriber subPose           = n.subscribe("/zed/zed_node/pose", 10, poseCallback);
  //publisher
  ros::Publisher local_pos_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 10); //mocap vision_pose /mavros/vision_pose/pose

 ros::Rate rate(100.0); 

 
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

 local_pos_pub.publish(loc_pos);

 // ros::spin();
 ros::spinOnce();
 rate.sleep();

}


  return 0;
}


