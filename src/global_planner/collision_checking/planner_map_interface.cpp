
#include <mav_planning/global_planner/collision_checking/planner_map_interface.h>

namespace mav_planning
{
    PlannerMapInterface::PlannerMapInterface(Type type)
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

    void PlannerMapInterface::updateMAVState(const SE3State& state)
    {
        _mav_shape.setTranslation(state.position);
        _mav_shape.setOrientation(state.rotation);
    }

    PlannerMapInterface::Type PlannerMapInterface::getType()
    {
        return _type;
    }

    void PlannerMapInterface::lockMap()
    {
        _map_locked = true;
    }

    void PlannerMapInterface::unlockMap()
    {
        _map_locked = false;
    }

}
