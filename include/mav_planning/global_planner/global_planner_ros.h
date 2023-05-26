#ifndef MAV_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_ROS
#define MAV_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_ROS

#include <mav_planning/common/ros/conversions.h>
#include "map_interfaces/custom_map_interface.h"
#include "map_interfaces/octomap_interface.h"
#include "global_planner.h"

typedef mav_planning::GlobalPlanner Planner;
typedef mav_planning::Custom3DMap Map;
typedef mav_planning::PlannerMapInterface MapInterface;
typedef mav_planning::CollisionGeometry Geometry;

namespace mav_planning
{
    class PlanManagerROS
    {
        public:
            PlanManagerROS(ros::NodeHandle nh, ros::NodeHandle nh2);
            ~PlanManagerROS(void){};

            void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goal);
            void startCB(const geometry_msgs::PoseStamped::ConstPtr& start);
        
            void visualizeMap();
            void visualizePlan();
            void setupPlanner();
            void planPath();

        private:
            ros::NodeHandle _nh, _nh_priv;
            ros::Publisher _path_pub, _path_vis_pub, _approx_path_vis_pub, _map_vis_pub;
            ros::Subscriber _goal_sub, _start_sub;
            std::shared_ptr<Planner> _planner;
            std::shared_ptr<Map> _map_ptr;
            std::shared_ptr<MapInterface> _map_interface_ptr;

            Planner::PlannerParams _planner_params;
            Map::RandomMapParams _rand_map_params;
            
            std::pair<Point, Point> _map_bounds;
            Geometry _mav_shape;
            FlatMAVState _start, _goal;
            FlatMAVStateList _path;

            void validateMapBounds();
            void processObstacles();
    };
}



#endif /* MAV_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_ROS */
