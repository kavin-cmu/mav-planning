#ifndef MAV_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER
#define MAV_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER

// Common Includes
#include "state_spaces/FlatMAVStateSpace.h"
#include "collision_checking/planner_map_interface.h"
#include "optimization_objectives/multi_optimisation_objective.h"
#include "collision_checking/state_validator.h"

#include <ompl/geometric/planners/prm/LazyPRMstar.h>
#include <ompl/geometric/planners/prm/LazyPRM.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
// RRT-based planners
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>

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

enum StateSpaceType {XYZ_STATE_SPACE, FLAT_MAV_STATE_SPACE, SE3_STATE_SPACE};

class GlobalPlanner{
    
    public:
        GlobalPlanner();

        ~GlobalPlanner(void) {};

        enum PlannerType{RRT, RRTStar, RRTConnect, InformedRRTStar, BITStar};
        
        struct PlannerParams
        {   
            PlannerType type;
            bool use_auto_range;
            float planner_range;
            float rewire_factor;
            float plan_time_lim;
            PlannerOptimParams optim_params;
            StateValidatorParams svc_params;
        };

        void setStartState(const FlatMAVState& start);
        void setGoalState(const FlatMAVState& goal);

        std::pair<Point, Point> getXYZBounds();
        void setMAVShape(CollisionGeometry shape);
        void setupPlanner(PlannerParams params);
        void planPath(const FlatMAVState& start, const FlatMAVState& goal);
        void registerMapInterface(std::shared_ptr<PlannerMapInterface>& map);
        void printPlanningData(std::string dir);
        FlatMAVStateList getPlannedPath(ob::PlannerStatus& type);

    private:
        
        // Planner
        size_t _dim = 3;
        ob::StateSpacePtr _space;
        std::shared_ptr<ob::RealVectorBounds> _bounds;
        std::pair<Point, Point> _bounds_eig;
        ob::SpaceInformationPtr _si;
        ob::ScopedStatePtr _start, _goal;
        ob::ProblemDefinitionPtr _pdef;
        ob::PlannerPtr _planner;
        FlatMAVStateList _path, _path_prev, _path_approx;
        ob::PathPtr _path_ptr;
        PlannerParams _params;
        bool _has_exact_soln = false;
        ob::PlannerStatus _soln_type;

        // Collision Checking Interface
        std::shared_ptr<PlannerMapInterface> _map;
        CollisionGeometry _mav_geometry;

        bool hasExactSolutionPTC();
        void setXYZBounds(const std::pair<Point, Point>& bounds);
        void intermediateSolnCB(const ob::Planner* planner, const std::vector<const ob::State *>& states, ob::Cost cost);
};

}

#endif /* MAV_PLANNING_GLOBAL_PLANNER_GLOBAL_PLANNER */
