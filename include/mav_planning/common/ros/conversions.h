#ifndef MAV_PLANNING_COMMON_UTILS
#define MAV_PLANNING_COMMON_UTILS

#include "../common.h"
#include "../collision_geometry.h"

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace mav_planning
{   

    namespace convert
    {
        geometry_msgs::Pose toPoseMsg(const Point& pos, const Quaternion& rot);
        geometry_msgs::Pose toPoseMsg(const FlatMAVState& state);
        visualization_msgs::Marker toMarkerMsg(const CollisionGeometry& geom);
        visualization_msgs::MarkerArray toMarkerArrayMsg(const FlatMAVStateList& path);
        Quaternion toEigenQuat(const float& yaw);  
    }
    


}
#endif /* MAV_PLANNING_COMMON_UTILS */
