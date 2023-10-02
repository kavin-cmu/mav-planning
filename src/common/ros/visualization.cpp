#include <mav_planning/common/ros/visualization.h>


namespace mav_planning::vis_utils
{   
    void halfAlpha(std_msgs::ColorRGBA& color)
    {
        color.a = 0.75;
        return;
    }
    
    void fullAlpha(std_msgs::ColorRGBA& color)
    {
        color.a = 1.0;
        return;
    }

    std_msgs::ColorRGBA getColor(Color color)
    {
        std_msgs::ColorRGBA result;

        switch(color)
        {
            case RED:
                result.r = 0.8;
                result.g = 0.1;
                result.b = 0.1;
                result.a = 1.0;
                break;
            case GREEN:
                result.r = 0.1;
                result.g = 0.8;
                result.b = 0.1;
                result.a = 1.0;
                break;
            case GREY:
                result.r = 0.9;
                result.g = 0.9;
                result.b = 0.9;
                result.a = 1.0;
                break;
            case DARK_GREY:
                result.r = 0.6;
                result.g = 0.6;
                result.b = 0.6;
                result.a = 1.0;
                break;
            case WHITE:
                result.r = 1.0;
                result.g = 1.0;
                result.b = 1.0;
                result.a = 1.0;
                break;
            case ORANGE:
                result.r = 1.0;
                result.g = 0.5;
                result.b = 0.0;
                result.a = 1.0;
                break;
            case TRANSLUCENT_LIGHT:
                result.r = 0.1;
                result.g = 0.1;
                result.b = 0.1;
                result.a = 0.1;
                break;
            case TRANSLUCENT:
                result.r = 0.1;
                result.g = 0.1;
                result.b = 0.1;
                result.a = 0.25;
                break;
            case TRANSLUCENT_DARK:
                result.r = 0.1;
                result.g = 0.1;
                result.b = 0.1;
                result.a = 0.5;
                break;
            case BLACK:
                result.r = 0.0;
                result.g = 0.0;
                result.b = 0.0;
                result.a = 1.0;
                break;
            case YELLOW:
                result.r = 1.0;
                result.g = 1.0;
                result.b = 0.0;
                result.a = 1.0;
                break;
            case BROWN:
                result.r = 0.597;
                result.g = 0.296;
                result.b = 0.0;
                result.a = 1.0;
                break;
            case PINK:
                result.r = 1.0;
                result.g = 0.4;
                result.b = 1;
                result.a = 1.0;
                break;
            case LIME_GREEN:
                result.r = 0.6;
                result.g = 1.0;
                result.b = 0.2;
                result.a = 1.0;
                break;
            case CLEAR:
                result.r = 1.0;
                result.g = 1.0;
                result.b = 1.0;
                result.a = 0.0;
                break;
            case PURPLE:
                result.r = 0.597;
                result.g = 0.0;
                result.b = 0.597;
                result.a = 1.0;
                break;
            case CYAN:
                result.r = 0.0;
                result.g = 1.0;
                result.b = 1.0;
                result.a = 1.0;
                break;
            case MAGENTA:
                result.r = 1.0;
                result.g = 0.0;
                result.b = 1.0;
                result.a = 1.0;
                break;
            
            case TRANSLUCENT_RED:
                result.r = 0.8;
                result.g = 0.1;
                result.b = 0.1;
                result.a = 0.25;
                break;
            case TRANSLUCENT_GREEN:
                result.r = 0.1;
                result.g = 0.8;
                result.b = 0.1;
                result.a = 0.25;
                break;
            case TRANSLUCENT_GREY:
                result.r = 0.9;
                result.g = 0.9;
                result.b = 0.9;
                result.a = 0.25;
                break;
            case TRANSLUCENT_DARK_GREY:
                result.r = 0.6;
                result.g = 0.6;
                result.b = 0.6;
                result.a = 0.25;
                break;
            case TRANSLUCENT_WHITE:
                result.r = 1.0;
                result.g = 1.0;
                result.b = 1.0;
                result.a = 0.25;
                break;
            case TRANSLUCENT_ORANGE:
                result.r = 1.0;
                result.g = 0.5;
                result.b = 0.0;
                result.a = 0.25;
                break;
    
            case TRANSLUCENT_BLACK:
                result.r = 0.0;
                result.g = 0.0;
                result.b = 0.0;
                result.a = 0.25;
                break;
            case TRANSLUCENT_YELLOW:
                result.r = 1.0;
                result.g = 1.0;
                result.b = 0.0;
                result.a = 0.25;
                break;
            case TRANSLUCENT_BROWN:
                result.r = 0.597;
                result.g = 0.296;
                result.b = 0.0;
                result.a = 0.25;
                break;
            case TRANSLUCENT_PINK:
                result.r = 1.0;
                result.g = 0.4;
                result.b = 1;
                result.a = 0.25;
                break;
            case TRANSLUCENT_LIME_GREEN:
                result.r = 0.6;
                result.g = 1.0;
                result.b = 0.2;
                result.a = 0.25;
                break;

            case TRANSLUCENT_PURPLE:
                result.r = 0.597;
                result.g = 0.0;
                result.b = 0.597;
                result.a = 0.25;
                break;
            case TRANSLUCENT_CYAN:
                result.r = 0.0;
                result.g = 1.0;
                result.b = 1.0;
                result.a = 0.25;
                break;
            case TRANSLUCENT_MAGENTA:
                result.r = 1.0;
                result.g = 0.0;
                result.b = 1.0;
                result.a = 0.25;
                break;
            
            case TRANSLUCENT_BLUE:
                result.r = 0.1;
                result.g = 0.25;
                result.b = 0.8;
                result.a = 0.25;
                break;

            case BLUE:
            
            default:
                result.r = 0.1;
                result.g = 0.25;
                result.b = 0.8;
                result.a = 1.0;
                break;
        }

        return result;
    }

    visualization_msgs::Marker getDisplayMarker(const CollisionGeometry& geom, 
                                                const Color& color = GREY,
                                                const uint8_t& id=0u,
                                                const std::string& ns="")
    {
        visualization_msgs::Marker msg;

        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.action = visualization_msgs::Marker::ADD;
        msg.id = id;
        msg.ns = ns;
        msg.color = getColor(color);

        msg.pose = mav_planning::convert::toPoseMsg(geom.getTranslation(), geom.getOrientation());
        
        CollisionGeometry::Type type = geom.getType();

        auto shape = geom.getShape();
        
        switch(type)
        {   
            case CollisionGeometry::Type::SPHERE:
            {
                msg.type = visualization_msgs::Marker::SPHERE;
                msg.scale.x = 2*shape[0];
                msg.scale.y = 2*shape[0]; 
                msg.scale.z = 2*shape[0]; 
                break;
            }

            case CollisionGeometry::Type::BOX:
            {
                msg.type = visualization_msgs::Marker::CUBE;
                msg.scale.x = shape[0];
                msg.scale.y = shape[1]; 
                msg.scale.z = shape[2]; 
                break;
            }

            case CollisionGeometry::Type::CYLINDER:
            {
                msg.type = visualization_msgs::Marker::CYLINDER;
                msg.scale.x = 2*shape[0];
                msg.scale.y = 2*shape[0]; 
                msg.scale.z =   shape[1]; 
                break;
            }

            case CollisionGeometry::Type::ELLIPSOID:
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

    visualization_msgs::Marker getDisplayMarker(const SE3StateList& path, 
                                                const Color& color=RED, 
                                                const double& width=0.25f,
                                                const uint8_t& id=0u,
                                                const std::string& ns="")
    {
        visualization_msgs::Marker path_msg;

        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        path_msg.action = visualization_msgs::Marker::ADD;

        path_msg.type = visualization_msgs::Marker::LINE_STRIP;
        path_msg.ns = ns;
        path_msg.id = id;

        path_msg.color = getColor(color);
        path_msg.scale.x = width;
        
        for(auto pts : path)
        {   
            geometry_msgs::Pose pose = mav_planning::convert::toPoseMsg(pts);
            path_msg.points.push_back(pose.position);
        }

        return path_msg;
    }

    visualization_msgs::Marker getDisplayMarker(const PointList& path, 
                                                const Color& color=RED, 
                                                const double& width=0.25f,
                                                const uint8_t& id=0u,
                                                const std::string& ns="")
    {
        visualization_msgs::Marker path_msg;

        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        path_msg.action = visualization_msgs::Marker::ADD;

        path_msg.type = visualization_msgs::Marker::LINE_STRIP;
        path_msg.ns = ns;
        path_msg.id = id;

        path_msg.color = getColor(color);
        path_msg.scale.x = width;
        
        for(auto pts : path)
        {   
            geometry_msgs::Point pt;
            
            pt.x = pts[0];
            pt.y = pts[1]; 
            pt.z = pts[2];
            
            path_msg.points.push_back(pt);
        }
        
        return path_msg;
    }

    visualization_msgs::Marker getDisplayMarker(const Point& point, 
                                                const Color& color, 
                                                const double& size=0.25f,
                                                const uint8_t& id=0u,
                                                const std::string& ns="")
    {
        visualization_msgs::Marker msg;

        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.action = visualization_msgs::Marker::ADD;
        msg.id = id;
        msg.ns = ns;
        msg.color = getColor(color);

        msg.pose.position.x = point[0];
        msg.pose.position.y = point[1];
        msg.pose.position.z = point[2];
        msg.pose.orientation.w = 1.0;
                
        msg.type = visualization_msgs::Marker::SPHERE;

        msg.scale.x = 2*size;
        msg.scale.y = 2*size; 
        msg.scale.z = 2*size; 

        return msg;
    }

    visualization_msgs::Marker getDisplayMarker(const fcl::OBBd& bbox, 
                                                const Color& color, 
                                                const uint8_t& id,
                                                const std::string& ns)
    {
        visualization_msgs::Marker msg;

        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.action = visualization_msgs::Marker::ADD;
        msg.id = id;
        msg.ns = ns;
        msg.color = getColor(color);

        msg.pose = convert::toPoseMsg(bbox.center(), Quaternion(bbox.axis));
                
        msg.type = visualization_msgs::Marker::CUBE;

        msg.scale.x = bbox.width();
        msg.scale.y = bbox.depth(); 
        msg.scale.z = bbox.height(); 

        return msg;
    }

    visualization_msgs::Marker getDisplayMarker(const fcl::AABBd& bbox, 
                                                const Color& color, 
                                                const uint8_t& id,
                                                const std::string& ns)
    {
        visualization_msgs::Marker msg;

        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.action = visualization_msgs::Marker::ADD;
        msg.id = id;
        msg.ns = ns;
        msg.color = getColor(color);

        msg.pose.position.x = bbox.center().x();
        msg.pose.position.y = bbox.center().y();
        msg.pose.position.z = bbox.center().z();
        msg.pose.orientation.w = 1.0;
                
        msg.type = visualization_msgs::Marker::CUBE;

        msg.scale.x = bbox.width();
        msg.scale.y = bbox.depth(); 
        msg.scale.z = bbox.height(); 

        return msg;
    }

    visualization_msgs::Marker getDisplayMarker(const AABBox& bbx, 
                                                const Color& color, 
                                                const uint8_t& id,
                                                const std::string& ns)
    {
        visualization_msgs::Marker msg;

        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        msg.action = visualization_msgs::Marker::ADD;
        msg.id = id;
        msg.ns = ns;
        msg.color = getColor(color);

        double dx, dy, dz;

        auto dr = bbx.max-bbx.min;

        msg.pose = convert::toPoseMsg(bbx.min+0.5*dr, Quaternion(Eigen::AngleAxisd(0.0, Point::UnitX())));
    
                
        msg.type = visualization_msgs::Marker::CUBE;

        msg.scale.x = dr[0];
        msg.scale.y = dr[1]; 
        msg.scale.z = dr[2]; 

        return msg;
    }

    // visualization_msgs::Marker getDisplayMarker(const std::vector<Point>& verts, 
    //                                             const Color& color, 
    //                                             const uint8_t& id,
    //                                             const std::string& ns)
    // {
    //     visualization_msgs::Marker msg;

    //     msg.header.frame_id = "map";
    //     msg.header.stamp = ros::Time::now();
    //     msg.action = visualization_msgs::Marker::ADD;
    //     msg.id = id;
    //     msg.ns = ns;
    //     msg.color = getColor(color);

    //     auto vert_list = verts;

    //     vert_list.push_back(verts[0]);

    //     for(auto vert : vert_list)
    //     {
    //         geometry_msgs::Point pt;
    //         pt.x = vert.x();
    //         pt.y = vert.y();
    //         pt.z = vert.z();
    //         msg.points.push_back(pt);
    //     }
    
                
    //     msg.type = visualization_msgs::Marker::LINE_STRIP;

    //     msg.scale.x = 0.10; 

    //     return msg;
    // }
}
