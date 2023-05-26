#ifndef GLOBAL_PLANNER_COLLISION_CHECKING_PLANNER_MAP_INTERFACE
#define GLOBAL_PLANNER_COLLISION_CHECKING_PLANNER_MAP_INTERFACE

#include <mav_planning/common/common.h>
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
            
            enum MapType {CUSTOM3D, OPENVDB, OCTOMAP, VOXBLOX, GRIDMAP};
            
            PlannerMapInterface(MapType type);     
            
            ~PlannerMapInterface(void){};

            friend class StateValidator;

            bool hasBounds();
            
            void setMAVShape(CollisionGeometry mav_shape);
            CollisionGeometry getMAVShape();
            void updateMAVState(const ob::State* state);
            std::pair<Point, Point> getMapBounds(){return _bounds;}
            MapType getMapType();
            
            virtual bool checkCollision(const CollisionGeometry& shape){};
            virtual float getMinClearance(const CollisionGeometry& shape){};
            virtual void setMapBounds(const std::pair<Point, Point>& bounds){};
        
        protected:
            MapType _type;
            std::pair<Point, Point> _bounds;
            bool _has_bounds = false;
        
        private:
            CollisionGeometry _mav_shape;
                    
    };

}

#endif /* GLOBAL_PLANNER_COLLISION_CHECKING_PLANNER_MAP_INTERFACE */
