#ifndef GLOBAL_PLANNER_MAP_INTERFACES_OCTOMAP_INTERFACE
#define GLOBAL_PLANNER_MAP_INTERFACES_OCTOMAP_INTERFACE

#include "../collision_checking/planner_map_interface.h"
#include <mav_planning/common/collision_geometry.h>

#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>
#include <octomap_ros/conversions.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>

namespace mav_planning
{

    class OctoMapInterface: public PlannerMapInterface
    {   
        public:
            OctoMapInterface(const ros::NodeHandle nh_priv, const  ros::NodeHandle nh):PlannerMapInterface(Type::OCTOMAP)
            {
                _nh_priv = nh_priv;
                _nh = nh;

                _nh_priv.param<double>("octomap_interface/map_res", _map_res, 0.20);

                _octomap_sub = _nh.subscribe("/octomap", 1 ,&OctoMapInterface::mapCB, this);
                _sdf_pub = _nh.advertise<visualization_msgs::Marker>("/sdf_map", 1, true);
                
                _octree = std::make_shared<octomap::OcTree>(_map_res);                
    
            }

            ~OctoMapInterface(void)
            {
                // delete _octree;
            }

            void mapCB(const octomap_msgs::Octomap::ConstPtr& map);
            void updateEDTMap(float max_dist, const Point& p_min, const Point& p_max);
            
            // Overriden methods from PlannerMapInterface
            bool checkCollision(const SE3State& state);
            double getMinClearance(const SE3State& state);
            bool hasInit = false;
        
        private:
            std::string _name = "OctoMapInterface";
            ros::NodeHandle _nh_priv, _nh;
            ros::Subscriber _octomap_sub;
            ros::Publisher _sdf_pub;
            std::shared_ptr<octomap::OcTree> _octree = NULL;
            std::shared_ptr<fcl::CollisionObjectd> _map_col_obj;
            std::shared_ptr<DynamicEDTOctomap> _edt_map_ptr;
            void displayEDTMap(double max_dist, octomap::point3d& min, octomap::point3d& max);
            unsigned _tree_depth;
            double _map_res;
            std::pair<Point, Point> _edt_bounds;
    };
}


#endif /* GLOBAL_PLANNER_MAP_INTERFACES_OCTOMAP_INTERFACE */
