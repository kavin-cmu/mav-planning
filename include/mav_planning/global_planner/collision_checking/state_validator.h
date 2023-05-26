#ifndef MAV_PLANNING_OMPL_INTERFACE_STATE_VALIDATOR
#define MAV_PLANNING_OMPL_INTERFACE_STATE_VALIDATOR

#include "../state_spaces/FlatMAVStateSpace.h"
#include "planner_map_interface.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>

namespace ob = ompl::base;
// namespace og = ompl::geometric;

namespace mav_planning

{
    struct StateValidatorParams
    {  
        float virtual_ceiling;
        float ground_clearance;
    };

    class StateValidator : public ob::StateValidityChecker
    {
        public:
            StateValidator(const ob::SpaceInformationPtr& si, const std::shared_ptr<PlannerMapInterface>& ptr, const StateValidatorParams& params) :
                ob::StateValidityChecker(si) {
                    _map_ptr = ptr;
                    _params = params;
                }
        
            bool isValid(const ob::State *state) const
            {  
                float state_z = state->as<FlatMAVStateSpace::StateType>()->getZ();

                if( state_z < _params.ground_clearance || state_z > _params.virtual_ceiling)
                {
                    return false;
                }
                    _map_ptr->updateMAVState(state);
                    return !_map_ptr->checkCollision(_map_ptr->_mav_shape);
            }

            double clearance(const ob::State *state) const
            {   
                _map_ptr->updateMAVState(state);
                return _map_ptr->getMinClearance(_map_ptr->_mav_shape);
            }
        
        private:
            std::shared_ptr<PlannerMapInterface> _map_ptr;
            StateValidatorParams _params;
    };

}




#endif /* MAV_PLANNING_OMPL_INTERFACE_STATE_VALIDATOR */
