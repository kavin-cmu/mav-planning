#ifndef MAV_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER
#define MAV_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER

// Common Includes
#include "mav_planning/common/logging.h"
#include "state_spaces/FlatMAVStateSpace.h"
#include "collision_checking/planner_map_interface.h"
#include "optimization_objectives/multi_optimisation_objective.h"
#include "collision_checking/state_validator.h"
#include "state_spaces/state_utils.h"

#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
// RRT-based planners
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTXstatic.h>
#include <mav_planning/global_planner/planners/RRTx.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
// #include <ompl/geometric/planners/info>


#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/PlannerDataStorage.h>
#include <ompl/base/PlannerDataGraph.h>

#include <ompl/util/Console.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;

namespace mav_planning{

class GlobalPlanner{
    
    public:
        GlobalPlanner(StateSpaceType type);

        ~GlobalPlanner(void) {};

        enum Type{RRT, RRTStar, RRTConnect, InformedRRTStar, BITStar, PRMStar, RRTXStatic, ABITStar};
        
        struct Params
        {   
            Type type;
            bool use_auto_range;
            bool smoothen;
            double planner_range;
            double interp_resolution;
            double rewire_factor;
            double plan_time_lim;
            bool use_full_plan_time;
            bool wait_for_exact_soln;
            PlannerOptimParams optim_params;
            StateValidatorParams svc_params;
        };

        void setStartState(const SE3State& start);
        void setGoalState(const SE3State& goal);
        
        SE3State getStartState();
        SE3State getGoalState();

        std::pair<Point, Point> getXYZBounds();
        void setMAVShape(CollisionGeometry& shape);
        void setupPlanner(Params params);
        void planPath(const SE3State& start, const SE3State& goal);
        void registerMapInterface(std::shared_ptr<PlannerMapInterface>& map);
        void printPlanningData(std::string dir);
        SE3StateList getPlannedPath(ob::PlannerStatus& type);

    private:
        
        // Planner
        std::string _name = "GlobalPlanner";
        size_t _dim = 3;
        ob::StateSpacePtr _space;
        std::shared_ptr<ob::RealVectorBounds> _bounds;
        std::pair<Point, Point> _bounds_eig;
        ob::SpaceInformationPtr _si;
        ob::ScopedStatePtr _start, _goal;
        ob::ProblemDefinitionPtr _pdef;
        ob::PlannerPtr _planner;
        SE3StateList _path, _path_prev, _path_approx;
        ob::PathPtr _path_ptr;
        Params _params;
        bool _has_exact_soln = false;
        ob::PlannerStatus _soln_type;
        StateSpaceType _spaceType;

        // Collision Checking Interface
        std::shared_ptr<PlannerMapInterface> _map;
        CollisionGeometry _mav_geometry;

        bool hasExactSolutionPTC();
        void setXYZBounds(const std::pair<Point, Point>& bounds);
        void intermediateSolnCB(const ob::Planner* planner, const std::vector<const ob::State *>& states, ob::Cost cost);
        void constrainState(SE3State& state);
};

}

#endif /* MAV_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER */
