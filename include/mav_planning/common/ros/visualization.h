#ifndef COMMON_ROS_VISUALIZATION
#define COMMON_ROS_VISUALIZATION

#include "../common.h"
#include "../collision_geometry.h"

#include <mav_planning/common/ros/conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>

namespace mav_planning::vis_utils
{   

        enum Color
        {
            BLACK = 0, BROWN = 1, BLUE = 2, CYAN = 3, GREY = 4,
            DARK_GREY = 5, GREEN = 6, LIME_GREEN = 7, MAGENTA = 8, ORANGE = 9,
            PURPLE = 10, RED = 11, PINK = 12, WHITE = 13, YELLOW = 14,
            TRANSLUCENT = 15, TRANSLUCENT_LIGHT = 16, TRANSLUCENT_DARK = 17, RAND = 18, CLEAR = 19,
            DEFAULT = 20  // i.e. 'do not change default color'
        };

        std_msgs::ColorRGBA getColor(Color color);
        void halfAlpha(std_msgs::ColorRGBA& color);
        void fullAlpha(std_msgs::ColorRGBA& color);
        

        visualization_msgs::Marker getDisplayMarker(const CollisionGeometry& geom, 
                                                    const Color& color,
                                                    const uint8_t& id,
                                                    const std::string& ns);

        visualization_msgs::Marker getDisplayMarker(const FlatMAVStateList& path, 
                                                    const Color& color, 
                                                    const float& width,
                                                    const uint8_t& id,
                                                    const std::string& ns);

        visualization_msgs::Marker getDisplayMarker(const PointList& path, 
                                                    const Color& color, 
                                                    const float& width,
                                                    const uint8_t& id,
                                                    const std::string& ns);

        visualization_msgs::Marker getDisplayMarker(const Point& point, 
                                                    const Color& color, 
                                                    const float& size,
                                                    const uint8_t& id,
                                                    const std::string& ns);

    // visualization_msgs::Marker getDisplayMarker(const Point& path, const Color& color, const float& size);
    // visualization_msgs::Marker getDisplayMarker(const Point& path, const Color& color, const float& size);

}

#endif /* COMMON_ROS_VISUALIZATION */
