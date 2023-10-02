#ifndef MAV_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_ROS
#define MAV_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_ROS

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <mav_planning/common/ros/rich_logging.h>
#include <mav_planning/common/ros/conversions.h>
#include <mav_planning/common/ros/visualization.h>
#include <quadrotor_msgs/PositionCommand.h>
// #include <mav_planning/common/ros/rich_logging.h>
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
            void startCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& start);
            void odomCB(const nav_msgs::Odometry::ConstPtr& odom);
        
            void visualizeMap();
            void visualizePlan();
            void setupPlanner();
            void planPath();

        private:
            ros::NodeHandle _nh, _nh_priv;
            ros::Publisher _path_pub, _path_vis_pub, _approx_path_vis_pub, _map_vis_pub, _cmd_pub;
            ros::Subscriber _goal_sub, _start_sub, _odom_sub;
            ros::Timer _controlLoop, _planLoop;
            std::shared_ptr<Planner> _planner;
            std::shared_ptr<Map> _map_ptr;
            std::shared_ptr<OctoMapInterface> _oct_map_ptr;

            std::shared_ptr<MapInterface> _map_interface_ptr;

            Planner::Params _planner_params;
            Map::RandomMapParams _rand_map_params;
            
            std::pair<Point, Point> _map_bounds;
            Geometry _mav_shape;
            SE3State _start, _goal;
            SE3StateList _curr_path;
            double _goal_z, _start_z;
            SE3State _curr_pose;
            Point _curr_vel;
            bool _flag_recvd_odom = false;
            bool _have_path = false, _have_new_path = false;
            bool _new_goal = false, _have_goal = false, _reached_goal = false;
            size_t _pt_idx = 0, _replan_cnt = 0;
            std::string _name = "PlanManagerROS";
            
            void validateMapBounds();
            void processObstacles();
            void runPlanner(const SE3State& start, const SE3State& goal);
            void controllerCB(const ros::TimerEvent& event);
            void plannerCB(const ros::TimerEvent& event);

    };
}



#endif /* MAV_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER_ROS */
