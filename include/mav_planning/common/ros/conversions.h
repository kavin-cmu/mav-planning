#ifndef COMMON_ROS_CONVERSIONS
#define COMMON_ROS_CONVERSIONS

#include "../common.h"
#include "../collision_geometry.h"

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace mav_planning::convert
{
    geometry_msgs::Pose toPoseMsg(const Point& pos, const Quaternion& rot);
    geometry_msgs::Pose toPoseMsg(const SE3State& state);
    Quaternion toEigenQuat(const double& yaw); 
    Quaternion toEigenQuat(const double& roll, const double& pitch, const double& yaw);
    Quaternion toEigenQuat(const geometry_msgs::Quaternion& quat) ;
    Eigen::Vector3d toEulerRPY(const Quaternion& quat);
    SE3State toSE3State(const geometry_msgs::Pose& msg);

    bool toCollisionGeometry(XmlRpc::XmlRpcValue& param, CollisionGeometry& shape);

}
#endif /* COMMON_ROS_CONVERSIONS */
