
#include <mav_planning/global_planner/collision_checking/planner_map_interface.h>

namespace mav_planning
{
    PlannerMapInterface::PlannerMapInterface(MapType type)
    {
        _type = type;
    }

    bool PlannerMapInterface::hasBounds()
    {
        return _has_bounds;
    }
    
    void PlannerMapInterface::setMAVShape(CollisionGeometry mav_shape)
    {   
        _mav_shape = mav_shape;
    }

    CollisionGeometry PlannerMapInterface::getMAVShape()
    {   
        return _mav_shape;
    }

    void PlannerMapInterface::updateMAVState(const ob::State* state)
    {
        _mav_shape.setTranslation(state->as<FlatMAVStateSpace::StateType>()->getXYZ());
        _mav_shape.setOrientation(state->as<FlatMAVStateSpace::StateType>()->getYawAsQuat());
    }

    PlannerMapInterface::MapType PlannerMapInterface::getMapType()
    {
        return _type;
    }

}
