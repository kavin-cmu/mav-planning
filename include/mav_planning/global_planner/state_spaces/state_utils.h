#ifndef GLOBAL_PLANNER_STATE_SPACES_UTILS
#define GLOBAL_PLANNER_STATE_SPACES_UTILS

#include "mav_planning/common/common.h"
#include "mav_planning/common/ros/conversions.h"

#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include "FlatMAVStateSpace.h"

namespace ob = ompl::base;

namespace mav_planning
{   
    enum StateSpaceType {XYZ_STATE_SPACE, FLAT_MAV_STATE_SPACE, SE3_STATE_SPACE};

    namespace convert
    {
        void toOMPLState(ob::ScopedStatePtr& out, const SE3State& in, const StateSpaceType& type);
        void fromOMPLState(const ob::ScopedStatePtr& in, SE3State& out, const StateSpaceType& type);
    }
}

#endif /* GLOBAL_PLANNER_STATE_SPACES_UTILS */
