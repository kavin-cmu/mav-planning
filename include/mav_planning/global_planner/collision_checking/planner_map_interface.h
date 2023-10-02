#ifndef GLOBAL_PLANNER_COLLISION_CHECKING_PLANNER_MAP_INTERFACE
#define GLOBAL_PLANNER_COLLISION_CHECKING_PLANNER_MAP_INTERFACE

#include <mav_planning/common/common.h>
#include <mav_planning/common/logging.h>
#include <mav_planning/common/collision_geometry.h>

#include "../state_spaces/FlatMAVStateSpace.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>

namespace ob = ompl::base;
// namespace og = ompl::geometric;


namespace mav_planning
{
    class PlannerMapInterface
    {
        public:
            
            enum Type {CUSTOM3D, OPENVDB, OCTOMAP, VOXBLOX, GRIDMAP};
            
            PlannerMapInterface(Type type);     
            
            ~PlannerMapInterface(void){};

            friend class StateValidator;

            bool hasBounds();
            
            void setMAVShape(CollisionGeometry mav_shape);
            CollisionGeometry getMAVShape();
            void updateMAVState(const SE3State& state);
            std::pair<Point, Point> getMapBounds(){return _bounds;}
            Type getType();
            void lockMap();
            void unlockMap();
            
            virtual bool checkCollision(const SE3State& state){};
            virtual double getMinClearance(const SE3State& state){};
            virtual double getTerrainHeight(const SE3State& state){};
            virtual void setMapBounds(const std::pair<Point, Point>& bounds){};
        
        protected:
            Type _type;
            std::pair<Point, Point> _bounds;
            bool _has_bounds = false;
            CollisionGeometry _mav_shape;
            bool _map_locked = false;
                    
    };

}

#endif /* GLOBAL_PLANNER_COLLISION_CHECKING_PLANNER_MAP_INTERFACE */
