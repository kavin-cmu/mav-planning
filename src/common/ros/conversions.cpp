#include <mav_planning/common/ros/conversions.h>

namespace mav_planning
{   
    namespace convert
    {
        geometry_msgs::Pose toPoseMsg(const Point& pos, const Quaternion& rot)
        {   
            geometry_msgs::Pose msg;

            msg.position.x = pos[0];
            msg.position.y = pos[1];
            msg.position.z = pos[2];
            msg.orientation.x = rot.x();
            msg.orientation.y = rot.y();
            msg.orientation.z = rot.z();
            msg.orientation.w = rot.w();

            return msg;
        }

        geometry_msgs::Pose toPoseMsg(const FlatMAVState& state)
        {
            Quaternion q(Eigen::AngleAxisd(state[3], Point::UnitZ()));
            Point p = {state[0], state[1], state[2]};

            return toPoseMsg(p, q);
        }

        visualization_msgs::Marker toMarkerMsg(const CollisionGeometry& geom)
        {
            visualization_msgs::Marker msg;

            msg.header.frame_id = "map";
            msg.header.stamp = ros::Time::now();
            msg.action = visualization_msgs::Marker::ADD;
            msg.ns = "map";
            msg.color.r = 0.5;
            msg.color.g = 0.5;
            msg.color.b = 0.5;
            msg.color.a = 0.90;

            msg.pose = toPoseMsg(geom.getTranslation(), geom.getOrientation());
            
            CollisionGeometry::GeometryType type = geom.getType();

            auto shape = geom.getShape();
            
            switch(type)
            {   
                case CollisionGeometry::GeometryType::SPHERE:
                {
                    msg.type = visualization_msgs::Marker::SPHERE;
                    msg.scale.x = 2*shape[0];
                    msg.scale.y = 2*shape[0]; 
                    msg.scale.z = 2*shape[0]; 
                    break;
                }

                case CollisionGeometry::GeometryType::BOX:
                {
                    msg.type = visualization_msgs::Marker::CUBE;
                    msg.scale.x = shape[0];
                    msg.scale.y = shape[1]; 
                    msg.scale.z = shape[2]; 
                    break;
                }

                case CollisionGeometry::GeometryType::CYLINDER:
                {
                    msg.type = visualization_msgs::Marker::CYLINDER;
                    msg.scale.x = 2*shape[0];
                    msg.scale.y = 2*shape[0]; 
                    msg.scale.z =   shape[1]; 
                    break;
                }

                case CollisionGeometry::GeometryType::ELLIPSOID:
                {
                    msg.type = visualization_msgs::Marker::SPHERE;
                    msg.scale.x = 2*shape[0];
                    msg.scale.y = 2*shape[1]; 
                    msg.scale.z = 2*shape[2]; 
                    break;
                }
                
                default:
                {
                    break;
                }
            }

            return msg;
        }

        visualization_msgs::MarkerArray toMarkerArrayMsg(const FlatMAVStateList& path)
        {
            visualization_msgs::Marker path_msg, pts_msg;
            visualization_msgs::MarkerArray comb_msg;

            path_msg.header.frame_id = pts_msg.header.frame_id = "map";
            path_msg.header.stamp = pts_msg.header.stamp = ros::Time::now();
            path_msg.action = pts_msg.action = visualization_msgs::Marker::ADD;

            pts_msg.type  = visualization_msgs::Marker::SPHERE;
            path_msg.type = visualization_msgs::Marker::LINE_STRIP;
            
            path_msg.ns = "path";
            pts_msg.ns = "points";
            
            path_msg.id = 0;

            path_msg.color.r = 1.0;
            pts_msg.color.g  = 1.0;
            path_msg.color.a = pts_msg.color.a = 1.0;

            path_msg.scale.x = 0.25;
            
            pts_msg.scale.x = path_msg.scale.x;
            pts_msg.scale.y = path_msg.scale.x;
            pts_msg.scale.z = path_msg.scale.x;

            size_t cnt = 0;
            for(auto pts : path)
            {   
                geometry_msgs::Pose pose = toPoseMsg(pts);

                path_msg.points.push_back(pose.position);
                pts_msg.pose = pose;
                pts_msg.id = cnt;
                comb_msg.markers.push_back(pts_msg);

                cnt++;
            }

            comb_msg.markers.push_back(path_msg);
            return comb_msg;
        }

        Quaternion toEigenQuat(const float& yaw)
        {
            return Quaternion(Eigen::AngleAxisd(yaw, Point::UnitZ()));
        }
    }   
}