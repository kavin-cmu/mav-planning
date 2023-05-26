
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
            std::cout<<"[GlobalPlanner::MultiCostObjective] Clearance objective enabled!\n";
        }
        else
        {
            std::cout<<"[GlobalPlanner::MultiCostObjective] Clearance objective disabled!\n";
        }
        opt->setCostToGoHeuristic(ob::goalRegionCostToGo);
    
        return ob::OptimizationObjectivePtr(opt);
    }
}