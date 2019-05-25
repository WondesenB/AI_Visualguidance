
// include
#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
// darknet_ros_msgs
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <local_mapping/detected_object.h>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/convert.h>
#include <kdl/frames.hpp>

#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <cstddef>
#include <math.h>
#include <vector>
#include <algorithm>

//
#include <tf/transform_broadcaster.h>
// #include <tf/transform_listener.h>

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
double rx, ry, rz, w;
double roll, pitch, yaw;


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
float  Z_min;
float  X;
float  Y;
float  Z;
float  distance;
float  area;
};

struct objects
{
vector<objects_> object;
}ob;


void zed2dronebase_transform (void);
// void boundingbox_callback(const darknet_ros_msgs::BoundingBoxes::ConstPtr&  msg);
// void camera_depth_callback(const sensor_msgs::Image::ConstPtr& msg);
// void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
// void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
// void imageRightRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg);
// void imageLeftRectifiedCallback(const sensor_msgs::Image::ConstPtr& msg);
// void pixelTo3DXYZ_callback(const sensor_msgs::PointCloud2 pointCL);
// void numOfdetectedObjetCallback(const std_msgs::Int8::ConstPtr& msg);

namespace tf2
{

/** \brief Convert a timestamped transform to the equivalent KDL data type.
 * \param t The transform to convert, as a geometry_msgs TransformedStamped message.
 * \return The transform message converted to an KDL Frame.
 */
inline
KDL::Frame transformToKDL(const geometry_msgs::TransformStamped& t)
  {
    return KDL::Frame(KDL::Rotation::Quaternion(t.transform.rotation.x, t.transform.rotation.y,
						t.transform.rotation.z, t.transform.rotation.w),
		      KDL::Vector(t.transform.translation.x, t.transform.translation.y, t.transform.translation.z));
  }

/** \brief Convert an KDL Frame to the equivalent geometry_msgs message type.
 * \param k The transform to convert, as an KDL Frame.
 * \return The transform converted to a TransformStamped message.
 */
inline
geometry_msgs::TransformStamped kdlToTransform(const KDL::Frame& k)
{
  geometry_msgs::TransformStamped t;
  t.transform.translation.x = k.p.x();
  t.transform.translation.y = k.p.y();
  t.transform.translation.z = k.p.z();
  k.M.GetQuaternion(t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w);
  return t;
}

// ---------------------
// Vector
// ---------------------
/** \brief Apply a geometry_msgs TransformStamped to an KDL-specific Vector type.
 * This function is a specialization of the doTransform template defined in tf2/convert.h.
 * \param t_in The vector to transform, as a timestamped KDL Vector data type.
 * \param t_out The transformed vector, as a timestamped KDL Vector data type.
 * \param transform The timestamped transform to apply, as a TransformStamped message.
 */
template <>
inline
  void doTransform(const tf2::Stamped<KDL::Vector>& t_in, tf2::Stamped<KDL::Vector>& t_out, const geometry_msgs::TransformStamped& transform)
  {
    t_out = tf2::Stamped<KDL::Vector>(transformToKDL(transform) * t_in, transform.header.stamp, transform.header.frame_id);
  }

/** \brief Convert a stamped KDL Vector type to a PointStamped message.
 * This function is a specialization of the toMsg template defined in tf2/convert.h.
 * \param in The timestamped KDL Vector to convert.
 * \return The vector converted to a PointStamped message.
 */
inline
geometry_msgs::PointStamped toMsg(const tf2::Stamped<KDL::Vector>& in)
{
  geometry_msgs::PointStamped msg;
  msg.header.stamp = in.stamp_;
  msg.header.frame_id = in.frame_id_;
  msg.point.x = in[0];
  msg.point.y = in[1];
  msg.point.z = in[2];
  return msg;
}

/** \brief Convert a PointStamped message type to a stamped KDL-specific Vector type.
 * This function is a specialization of the fromMsg template defined in tf2/convert.h
 * \param msg The PointStamped message to convert.
 * \param out The point converted to a timestamped KDL Vector.
 */
inline
void fromMsg(const geometry_msgs::PointStamped& msg, tf2::Stamped<KDL::Vector>& out)
{
  out.stamp_ = msg.header.stamp;
  out.frame_id_ = msg.header.frame_id;
  out[0] = msg.point.x;
  out[1] = msg.point.y;
  out[2] = msg.point.z;
}


}