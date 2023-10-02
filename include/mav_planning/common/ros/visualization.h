#ifndef COMMON_ROS_VISUALIZATION
#define COMMON_ROS_VISUALIZATION

#include "../common.h"
#include "../collision_geometry.h"

#include <mav_planning/global_planner/map_interfaces/octomap_interface.h>
#include <mav_planning/common/ros/conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

namespace mav_planning::vis_utils
{   

        enum Color
        {
            BLACK = 0,
            BROWN = 1,
            BLUE = 2,
            CYAN = 3,
            GREY = 4,
            DARK_GREY = 5,
            GREEN = 6,
            LIME_GREEN = 7,
            MAGENTA = 8,
            ORANGE = 9,
            PURPLE = 10,
            RED = 11,
            PINK = 12,
            WHITE = 13,
            YELLOW = 14,
            TRANSLUCENT = 15,
            TRANSLUCENT_LIGHT = 16,
            TRANSLUCENT_DARK = 17,
            CLEAR = 18,
            TRANSLUCENT_BLACK,
            TRANSLUCENT_BROWN,
            TRANSLUCENT_BLUE,
            TRANSLUCENT_CYAN,
            TRANSLUCENT_GREY,
            TRANSLUCENT_DARK_GREY ,
            TRANSLUCENT_GREEN ,
            TRANSLUCENT_LIME_GREEN ,
            TRANSLUCENT_MAGENTA ,
            TRANSLUCENT_ORANGE ,
            TRANSLUCENT_PURPLE ,
            TRANSLUCENT_RED ,
            TRANSLUCENT_PINK ,
            TRANSLUCENT_WHITE ,
            TRANSLUCENT_YELLOW
            // i.e. 'do not change default color'
        };

        std_msgs::ColorRGBA getColor(Color color);
        void halfAlpha(std_msgs::ColorRGBA& color);
        void fullAlpha(std_msgs::ColorRGBA& color);
        

        visualization_msgs::Marker getDisplayMarker(const CollisionGeometry& geom, 
                                                    const Color& color,
                                                    const uint8_t& id,
                                                    const std::string& ns);

        visualization_msgs::Marker getDisplayMarker(const SE3StateList& path, 
                                                    const Color& color, 
                                                    const double& width,
                                                    const uint8_t& id,
                                                    const std::string& ns);

        visualization_msgs::Marker getDisplayMarker(const PointList& path, 
                                                    const Color& color, 
                                                    const double& width,
                                                    const uint8_t& id,
                                                    const std::string& ns);

        visualization_msgs::Marker getDisplayMarker(const Point& point, 
                                                    const Color& color, 
                                                    const double& size,
                                                    const uint8_t& id,
                                                    const std::string& ns);
        
        visualization_msgs::Marker getDisplayMarker(const fcl::OBBd& bbox, 
                                                    const Color& color, 
                                                    const uint8_t& id,
                                                    const std::string& ns);
        
        visualization_msgs::Marker getDisplayMarker(const fcl::AABBd& bbox, 
                                                    const Color& color, 
                                                    const uint8_t& id,
                                                    const std::string& ns);

        visualization_msgs::Marker getDisplayMarker(const AABBox& bbx, 
                                                    const Color& color, 
                                                    const uint8_t& id,
                                                    const std::string& ns);

        // visualization_msgs::Marker getDisplayMarker(const std::vector<Point>& verts, 
        //                                             const Color& color, 
        //                                             const uint8_t& id,
        //                                             const std::string& ns);

        // visualization_msgs::Marker getDisplayMarker(std::shared_ptr<DynamicEDTOctomap>& sdf, 
        //                                             const uint8_t& id,
        //                                             const std::string& ns);

}

#endif /* COMMON_ROS_VISUALIZATION */
