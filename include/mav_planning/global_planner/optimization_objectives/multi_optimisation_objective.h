#ifndef MAV_OMPL_PLANNING_OMPL_INTERFACE_MULTI_OPTIMISATION_OBJECTIVE
#define MAV_OMPL_PLANNING_OMPL_INTERFACE_MULTI_OPTIMISATION_OBJECTIVE

#include "../state_spaces/FlatMAVStateSpace.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>

namespace ob = ompl::base;

namespace mav_planning
{   

    struct PlannerOptimParams
    {  
        bool en_clearance;
        float k_clearance, k_path_length;
    };

    class ClearanceObjective : public ob::StateCostIntegralObjective
    {
        public:
            ClearanceObjective(const ob::SpaceInformationPtr& si) :
                ob::StateCostIntegralObjective(si, false)
            {
            }
        
            ob::Cost stateCost(const ob::State* s) const
            {   
                return ob::Cost(1 / si_->getStateValidityChecker()->clearance(s));
            }
    };

    ob::OptimizationObjectivePtr getMultiOptimisationObjective(const ob::SpaceInformationPtr& si, 
                                                               const PlannerOptimParams& params);
}

#endif /* MAV_OMPL_PLANNING_OMPL_INTERFACE_MULTI_OPTIMISATION_OBJECTIVE */
