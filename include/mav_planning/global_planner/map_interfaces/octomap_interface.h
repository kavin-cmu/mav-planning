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

namespace mav_planning
{

    class OctoMapInterface: public PlannerMapInterface
    {   
        public:
            OctoMapInterface(const ros::NodeHandle nh_priv, const  ros::NodeHandle nh):PlannerMapInterface(MapType::OCTOMAP)
            {
                _nh_priv = nh_priv;
                _nh = nh;

                _nh_priv.param<float>("octomap_interface/map_res", _map_res, 0.20);
                
                _octree = std::make_shared<octomap::OcTree>(_map_res);                
    
            }

            ~OctoMapInterface(void)
            {
                // delete _octree;
            }

            // Overriden methods from PlannerMapInterface
            bool checkCollision(const CollisionGeometry& shape);
            float getMinClearance(const CollisionGeometry& shape);
            void updateEDTMap();
        
        private:
            std::shared_ptr<octomap::OcTree> _octree = NULL;
            std::shared_ptr<fcl::CollisionObjectf> _map_col_obj;
            ros::NodeHandle _nh_priv, _nh;
            ros::Subscriber _octomap_sub;
            unsigned _tree_depth;
            std::shared_ptr<DynamicEDTOctomap> _edt_map_ptr;
            float _map_res;
    };
}


#endif /* GLOBAL_PLANNER_MAP_INTERFACES_OCTOMAP_INTERFACE */
