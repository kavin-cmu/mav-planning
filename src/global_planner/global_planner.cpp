#include <mav_planning/global_planner/global_planner.h>

namespace mav_planning
{
GlobalPlanner::GlobalPlanner(StateSpaceType spaceType)
{
  _spaceType = spaceType;

  switch (spaceType)
  {
    case StateSpaceType::XYZ_STATE_SPACE: {
      _space = std::make_shared<ob::RealVectorStateSpace>(3);
      INFO(_name, "Using XYZ state-space");
      break;
    }

    case StateSpaceType::FLAT_MAV_STATE_SPACE: {
      INFO(_name, "Using Flat MAV state-space");
      _space = std::make_shared<FlatMAVStateSpace>();
      break;
    }

    case StateSpaceType::SE3_STATE_SPACE: {
      INFO(_name, "Using SE3 state-space");
      _space = std::make_shared<ob::SE3StateSpace>();
      break;
    }

    default:
      WARN(_name, "Invalid state-space setting! Defaulting to XYZ state-space");
      _spaceType = StateSpaceType::XYZ_STATE_SPACE;
      _space = std::make_shared<ob::RealVectorStateSpace>(3);
      break;
  }

  // Only log msgs with WARN severity and above
  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);

  _si = std::make_shared<ob::SpaceInformation>(_space);
  _bounds = std::make_shared<ob::RealVectorBounds>(_dim);
  _start = std::make_shared<ob::ScopedState<>>(_space);
  _goal = std::make_shared<ob::ScopedState<>>(_space);
}

void GlobalPlanner::setStartState(const SE3State& start)
{ 
  SE3State s = start;
  constrainState(s);
  convert::toOMPLState(_start, s, _spaceType);
}

void GlobalPlanner::setGoalState(const SE3State& goal)
{
  SE3State s = goal;
  constrainState(s);
  convert::toOMPLState(_goal, s, _spaceType);
}

SE3State GlobalPlanner::getStartState()
{
  SE3State start;
  convert::fromOMPLState(_start, start, _spaceType);
  return start;
}

SE3State GlobalPlanner::getGoalState()
{
  SE3State goal;
  convert::fromOMPLState(_goal, goal, _spaceType);
  return goal;
}

void GlobalPlanner::setXYZBounds(const std::pair<Point, Point>& bound)
{
  _bounds->resize(_dim);

  _bounds_eig = bound;

  for (size_t i = 0; i < _dim; i++)
  {
    _bounds->setLow(i, bound.first[i]);
    _bounds->setHigh(i, bound.second[i]);
  }

  switch (_spaceType)
  {
    case StateSpaceType::XYZ_STATE_SPACE: {
      _space->as<ob::RealVectorStateSpace>()->setBounds(*_bounds);
      break;
    }

    case StateSpaceType::FLAT_MAV_STATE_SPACE: {
      _space->as<FlatMAVStateSpace>()->setBounds(*_bounds);
      break;
    }

    case StateSpaceType::SE3_STATE_SPACE: {
      _space->as<ob::SE3StateSpace>()->setBounds(*_bounds);
      break;
    }

    default:
      throw ompl::Exception(_name + ": Invalid state space type provided!");
      break;
  }
}

std::pair<Point, Point> GlobalPlanner::getXYZBounds()
{
  return _bounds_eig;
}

void GlobalPlanner::constrainState(SE3State& state)
{
  state.position.x() = 0.90*std::max(_bounds_eig.first.x(), std::min(_bounds_eig.second.x(), state.position.x()));
  state.position.y() = 0.90*std::max(_bounds_eig.first.y(), std::min(_bounds_eig.second.y(), state.position.y()));
  state.position.z() = 0.90*std::max(_bounds_eig.first.z(), std::min(_bounds_eig.second.z(), state.position.z()));
}

void GlobalPlanner::setMAVShape(CollisionGeometry& shape)
{
  _mav_geometry = shape;
  _map->setMAVShape(shape);
}

void GlobalPlanner::registerMapInterface(std::shared_ptr<PlannerMapInterface>& map)
{
  _map = map;

  if (!_map->hasBounds())
  {
    throw ompl::Exception("[" + _name + "] Invalid bounds provided by MapInterface!");
  }

  setXYZBounds(_map->getMapBounds());
}

bool GlobalPlanner::hasExactSolutionPTC()
{
  return _has_exact_soln;
}

void GlobalPlanner::intermediateSolnCB(const ob::Planner* planner, const std::vector<const ob::State*>& states,
                                       ob::Cost cost)
{
  _has_exact_soln = true;
  // ROS_WARN("BRUH");
}

void GlobalPlanner::setupPlanner(Params param)
{
  // planner params
  _params = param;

  // setup the State Validity Checker
  _si->setStateValidityChecker(
      ob::StateValidityCheckerPtr(new StateValidator(_si, _map, _params.svc_params, _spaceType)));

  // Set state interpolation resolution for validity checking
  double max_dist = _si->getMaximumExtent();
  double frac = param.interp_resolution / max_dist;
  _si->getStateSpace()->as<ob::RealVectorStateSpace>()->setLongestValidSegmentFraction(frac);

  //
  _pdef = std::make_shared<ob::ProblemDefinition>(_si);
  // set optimization objective
  _pdef->setOptimizationObjective(getMultiOptimisationObjective(_si, _params.optim_params));

  // Intermediate soln callback function
  ob::ReportIntermediateSolutionFn interSolnCBFn = std::bind(
      &GlobalPlanner::intermediateSolnCB, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3);

  _pdef->setIntermediateSolutionCallback(interSolnCBFn);

  switch (_params.type)
  {
    case Type::RRT: {
      INFO(_name, "Selecting RRT Planner");
      _planner = std::make_shared<og::RRT>(_si);
      if (!_params.use_auto_range)
      {
        _planner->as<og::RRT>()->setRange(_params.planner_range);
      }
      break;
    }
    case Type::RRTStar: {
      INFO(_name, "Selecting RRTStar Planner");
      _planner = std::make_shared<og::RRTstar>(_si);
      if (!_params.use_auto_range)
      {
        _planner->as<og::RRTstar>()->setRange(_params.planner_range);
      }
      break;
    }
    case Type::RRTConnect: {
      INFO(_name, "Selecting RRTConnect Planner");
      _planner = std::make_shared<og::RRTConnect>(_si);
      if (!_params.use_auto_range)
      {
        _planner->as<og::RRTConnect>()->setRange(_params.planner_range);
      }
      break;
    }
    case Type::InformedRRTStar: {
      INFO(_name, "Selecting InformedRRTStar Planner");
      _planner = std::make_shared<og::InformedRRTstar>(_si);
      if (!_params.use_auto_range)
      {
        _planner->as<og::InformedRRTstar>()->setRange(_params.planner_range);
      }
      break;
    }
    case Type::BITStar: {
      INFO(_name, "Selecting BITStar Planner");
      _planner = std::make_shared<og::BITstar>(_si);
      break;
    }
    case Type::PRMStar: {
      INFO(_name, "Selecting PRMStar Planner");
      _planner = std::make_shared<og::PRMstar>(_si);
      break;
    }
    case Type::RRTXStatic: {
      INFO(_name, "Selecting RRTXStatic Planner");
      _planner = std::make_shared<og::RRTx>(_si);
      break;
    }
    default: {
      WARN(_name, "Incorrect planner type provided! Defaulting to BITStar");
      _planner = std::make_shared<og::BITstar>(_si);
      _params.type = Type::BITStar;
      break;
    }
  }
}

void GlobalPlanner::planPath(const SE3State& start, const SE3State& goal)
{
  
  _map->lockMap();

  setXYZBounds(_map->getMapBounds());
  
  DEBUG(_name, "Start: (%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf)", start.position[0], start.position[1],
        start.position[2], start.rotation.x(), start.rotation.y(), start.rotation.z(), start.rotation.w());

  DEBUG(_name, "Goal: (%.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf, %.2lf)", goal.position[0], goal.position[1],
        goal.position[2], goal.rotation.x(), goal.rotation.y(), goal.rotation.z(), goal.rotation.w());

  auto t_i = std::chrono::steady_clock::now();

  _pdef->clearGoal();
  _pdef->clearStartStates();
  _pdef->clearSolutionPaths();

  _has_exact_soln = false;
  
  if (_params.type == Type::PRMStar)
  {
    _planner->as<og::PRMstar>()->clearQuery();
  }
  else
  {
    _planner->clear();
  }

  setStartState(start);
  setGoalState(goal);

  _pdef->setStartAndGoalStates(*_start.get(), *_goal.get());
  // std::cout<<"LMAO 5");

  _planner->setProblemDefinition(_pdef);
  // std::cout<<"LMAO 6");

  _planner->setup();
  // std::cout<<"LMAO 7");

  ob::PlannerTerminationConditionFn exactSolnPTC = std::bind(&GlobalPlanner::hasExactSolutionPTC, this);
  ob::PlannerTerminationCondition timedPTC = ob::timedPlannerTerminationCondition(_params.plan_time_lim);
  auto multiPTC = ob::plannerOrTerminationCondition(exactSolnPTC, timedPTC);

  if (_params.use_full_plan_time)
  {
    _soln_type = _planner->solve(timedPTC);
  }
  else if (_params.wait_for_exact_soln)
  {
    ob::PlannerTerminationCondition timedPTC = ob::timedPlannerTerminationCondition(100.0);
    auto multiPTC2 = ob::plannerOrTerminationCondition(exactSolnPTC, timedPTC);
    _soln_type = _planner->solve(multiPTC2);
  }
  else
  {
    _soln_type = _planner->solve(multiPTC);
  }

  if (_soln_type == ob::PlannerStatus::EXACT_SOLUTION || _soln_type == ob::PlannerStatus::APPROXIMATE_SOLUTION)
  {  // if success
    INFO(_name, "Found solution!");

    auto t_f = std::chrono::steady_clock::now();

    // double cost;
    int64_t dt_plan = 0, dt_shorten = 0, dt_smoothen = 0;

    dt_plan = std::chrono::duration_cast<std::chrono::milliseconds>(t_f - t_i).count();

    _path.clear();

    _path_ptr = _pdef->getSolutionPath();

    og::PathGeometric* path = _path_ptr->as<og::PathGeometric>();

    auto cost = _path_ptr->cost(_pdef->getOptimizationObjective());

    og::PathSimplifier path_simp(_si, _pdef->getGoal(), _pdef->getOptimizationObjective());

    t_i = std::chrono::steady_clock::now();
    path_simp.reduceVertices(*path, 10);
    path_simp.shortcutPath(*path);
    path_simp.collapseCloseVertices(*path, 2);
    t_f = std::chrono::steady_clock::now();

    dt_shorten = std::chrono::duration_cast<std::chrono::milliseconds>(t_f - t_i).count();

    // INFO_SPECIAL(_name, "[dt_shorten: %ld ms]", dt_shorten);

    if (_params.smoothen)
    {
      t_i = std::chrono::steady_clock::now();
      path_simp.smoothBSpline(*path, 5);
      t_f = std::chrono::steady_clock::now();
      dt_smoothen = std::chrono::duration_cast<std::chrono::milliseconds>(t_f - t_i).count();
    }

    INFO_SPECIAL(_name, "[cost: %.2f][dt_plan: %ldms][dt_shorten: %ldms][dt_smoothen: %ldms]", cost.value(), dt_plan,
                 dt_shorten, dt_smoothen);

    size_t num_states = path->getStateCount();
    _path.resize(num_states);

    auto states = path->getStates();
    // auto a = static_cast<ob::ScopedStatePtr>(((*states[0]).as<ob::ScopedState<>>()));
    for (size_t i = 0; i < num_states; i++)
    {
      convert::fromOMPLState(std::make_shared<ob::ScopedState<>>(_space, states[i]), _path[i], _spaceType);
    }
  }
  else
  {
    ERROR(_name, "Planning failed!");
  }
  _map->unlockMap();
}

SE3StateList GlobalPlanner::getPlannedPath(ob::PlannerStatus& type)
{
  type = _soln_type;
  return _path;
}

void GlobalPlanner::printPlanningData(std::string dir)
{
  std::fstream fh, gh;

  // First write the planned path data
  fh.open(dir + "/path.csv", std::ios::out);

  if (!fh)
  {
    ERROR(_name, "Path data file not created!");
  }
  else
  {
    for (auto pt : _path)
    {
      auto p = pt.position;
      auto r = pt.rotation;
      fh << p[0] << " , " << p[1] << " , " << p[2] << " , " << r.x() << " , " << r.y() << " , " << r.z() << " , "
         << r.w() << "\n";
    }
    INFO(_name, "Path data file created successfully!");
    fh.close();
  }

  // Then write the graph data
  fh.open(dir + "/data.graphml", std::ios::out);
  if (!fh)
  {
    ERROR(_name, "Graph data file not created!");
  }
  else
  {
    ob::PlannerData data(_planner->getSpaceInformation());
    _planner->getPlannerData(data);
    data.printGraphML(fh);
    INFO(_name, "Graph data file created successfully!");
    fh.close();
  }

  // Then write the robot geometry data

  fh.open(dir + "/robot_model.csv", std::ios::out);
  if (!fh)
  {
    ERROR(_name, "Robot data file not created!");
  }
  else
  {
    _mav_geometry.print(fh);
    INFO(_name, "Robot data file created successfully!");
    fh.close();
  }
}

}  // namespace mav_planning