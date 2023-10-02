#ifndef GLOBAL_PLANNER_COLLISION_CHECKING_STATE_VALIDATOR
#define GLOBAL_PLANNER_COLLISION_CHECKING_STATE_VALIDATOR

#include "../state_spaces/state_utils.h"
#include "planner_map_interface.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateValidityChecker.h>

namespace ob = ompl::base;
// namespace og = ompl::geometric;

namespace mav_planning

{
    struct StateValidatorParams
    {  
        double virtual_ceiling;
        double ground_clearance;
    };

    class StateValidator : public ob::StateValidityChecker
    {
        public:
            StateValidator(const ob::SpaceInformationPtr& si, const std::shared_ptr<PlannerMapInterface>& ptr, 
                           const StateValidatorParams& params, const StateSpaceType& spaceType) :
            ob::StateValidityChecker(si) 
            {
                _map_ptr = ptr;
                _params = params;
                _type = spaceType;
            }
        
            bool isValid(const ob::State *state) const
            {  
                SE3State eig_state;
                convert::fromOMPLState(std::make_shared<ob::ScopedState<>>(si_->getStateSpace(), state), eig_state, _type);

                // terrain clearance checks
                if(eig_state.position[2]>_params.virtual_ceiling || eig_state.position[2]<_params.ground_clearance)
                {
                    return false;
                }

                return !_map_ptr->checkCollision(eig_state);
            }

            double clearance(const ob::State *state) const
            {   
                SE3State eig_state;
                convert::fromOMPLState(std::make_shared<ob::ScopedState<>>(si_->getStateSpace(), state), eig_state, _type);
                return _map_ptr->getMinClearance(eig_state);
            }
        
        private:
            std::shared_ptr<PlannerMapInterface> _map_ptr;
            StateValidatorParams _params;
            StateSpaceType _type;
    };

}




#endif /* GLOBAL_PLANNER_COLLISION_CHECKING_STATE_VALIDATOR */
