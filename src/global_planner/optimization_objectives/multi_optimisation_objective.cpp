
#include <mav_planning/global_planner/optimization_objectives/multi_optimisation_objective.h>

namespace mav_planning
{
    ob::OptimizationObjectivePtr getMultiOptimisationObjective(const ob::SpaceInformationPtr& si,
                                                               const PlannerOptimParams& params)
    {
        ob::OptimizationObjectivePtr lengthObj(new ob::PathLengthOptimizationObjective(si));
        ob::OptimizationObjectivePtr clearObj(new ClearanceObjective(si));
        ob::MultiOptimizationObjective* opt = new ob::MultiOptimizationObjective(si);
        
        opt->addObjective(lengthObj, params.k_path_length);
        if(params.en_clearance)
        {   
            opt->addObjective(clearObj, params.k_clearance);
            INFO("MultiOptimisationObjective", "Clearance objective enabled!");
        }
        else
        {
            INFO("MultiOptimisationObjective", "Clearance objective disabled!");
        }
        opt->setCostToGoHeuristic(ob::goalRegionCostToGo);
    
        return ob::OptimizationObjectivePtr(opt);
    }
}