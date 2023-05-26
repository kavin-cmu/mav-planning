#include <mav_planning/global_planner/global_planner.h>

namespace mav_planning
{ 

    GlobalPlanner::GlobalPlanner()
    {   
        // switch (spaceType)
        // {
        //     case StateSpaceType::XYZ_STATE_SPACE:
        //     {
        //         _space  = ob::StateSpacePtr(new ob::RealVectorStateSpace(3));
        //         break;
        //     }

        //     case StateSpaceType::FLAT_MAV_STATE_SPACE:
        //     {
        //         break;
        //     }

        //     default:
        //         break;
        // }
        
        // Only log msgs with WARN severity and above
        ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);
        
        _space  = std::make_shared<FlatMAVStateSpace>();
        _si     = std::make_shared<ob::SpaceInformation>(_space);
        _bounds = std::make_shared<ob::RealVectorBounds>(_dim);
        _start  = std::make_shared<ob::ScopedState<>>(_space);
        _goal   = std::make_shared<ob::ScopedState<>>(_space);
    } 

    void GlobalPlanner::setStartState(const FlatMAVState& start)
    {
        _start->get()->as<FlatMAVStateSpace::StateType>()->setXYZYaw(start);
        // switch (spaceType)
        // {
        //     case StateSpaceType::XYZ_STATE_SPACE:
        //     {
        //         _start->get()->as<ob::RealVectorStateSpace::StateType>()->;
        //     }

        //     case StateSpaceType::FLAT_MAV_STATE_SPACE:
        //     {
        //         break;
        //     }

        //     default:
        //         break;
        // }
    }

    void GlobalPlanner::setGoalState(const FlatMAVState& goal)
    {
        _goal->get()->as<FlatMAVStateSpace::StateType>()->setXYZYaw(goal);
    }

    void GlobalPlanner::setXYZBounds(const std::pair<Point, Point>& bound)
    {       
        _bounds->resize(_dim);

        _bounds_eig = bound;
        
        for(size_t i=0; i<_dim; i++)
        {
            _bounds->setLow(i, bound.first[i]);
            _bounds->setHigh(i, bound.second[i]);
        }

        _space->as<FlatMAVStateSpace>()->setBounds(*_bounds);
    }

    std::pair<Point, Point> GlobalPlanner::getXYZBounds()
    {       
        return _bounds_eig;
    }

    void GlobalPlanner::setMAVShape(CollisionGeometry shape)
    {        
        _mav_geometry = shape;
        _map->setMAVShape(shape);
    }

    void GlobalPlanner::registerMapInterface(std::shared_ptr<PlannerMapInterface>& map)
    {
        _map = map;

        if(!_map->hasBounds())
        {
            throw ompl::Exception("Invalid bounds provided by MapInterface!");
        }
        
        setXYZBounds(_map->getMapBounds());
    }

    bool GlobalPlanner::hasExactSolutionPTC()
    {
        return _has_exact_soln;
    }

    void GlobalPlanner::intermediateSolnCB(const ob::Planner* planner, const std::vector<const ob::State *>& states, ob::Cost cost)
    {
        _has_exact_soln = true;
    }

    void GlobalPlanner::setupPlanner(PlannerParams param)
    {
        _params = param;
        
        _si->setStateValidityChecker(ob::StateValidityCheckerPtr(new StateValidator(_si, _map, _params.svc_params)));

        _pdef = std::make_shared<ob::ProblemDefinition>(_si);

        _pdef->setOptimizationObjective(getMultiOptimisationObjective(_si, _params.optim_params));

        ob::ReportIntermediateSolutionFn interSolnCBFn = std::bind(&GlobalPlanner::intermediateSolnCB, 
                                                                    this, 
                                                                    std::placeholders::_1, 
                                                                    std::placeholders::_2, 
                                                                    std::placeholders::_3);

        _pdef->setIntermediateSolutionCallback(interSolnCBFn);

        switch (_params.type)
        {
            case PlannerType::RRT:
            {   
                std::cout<<"[GlobalPlanner] Selecting RRT Planner\n";
                _planner = std::make_shared<og::RRT>(_si);
                if(!_params.use_auto_range)
                {
                    _planner->as<og::RRT>()->setRange(_params.planner_range);
                }
                break;
            }
            case PlannerType::RRTStar:
            {   
                std::cout<<"[GlobalPlanner] Selecting RRTStar Planner\n";
                _planner = std::make_shared<og::RRTstar>(_si);
                if(!_params.use_auto_range)
                {
                    _planner->as<og::RRTstar>()->setRange(_params.planner_range);
                }
                break;
            }
            case PlannerType::RRTConnect:
            {   
                std::cout<<"[GlobalPlanner] Selecting RRTConnect Planner\n";
                _planner = std::make_shared<og::RRTConnect>(_si);
                if(!_params.use_auto_range)
                {
                    _planner->as<og::RRTConnect>()->setRange(_params.planner_range);
                }
                break;
            }
            case PlannerType::InformedRRTStar:
            {   
                std::cout<<"[GlobalPlanner] Selecting InformedRRTstar Planner\n";
                _planner = std::make_shared<og::InformedRRTstar>(_si);
                if(!_params.use_auto_range)
                {
                    _planner->as<og::InformedRRTstar>()->setRange(_params.planner_range);
                }
                break;
            }
            case PlannerType::BITStar:
            {   
                std::cout<<"[GlobalPlanner] Selecting BITStar Planner\n";
                _planner = std::make_shared<og::BITstar>(_si);
                break;
            }
            default:
            {   
                throw ompl::Exception("[GlobalPlanner] Incorrect planner type provided");
                break;
            }
        }

    }

    void GlobalPlanner::planPath(const FlatMAVState& start, const FlatMAVState& goal)
    {   
        auto t_i = std::chrono::steady_clock::now();

        _has_exact_soln = false;
        _pdef->clearGoal();
        _pdef->clearStartStates();
        _pdef->clearSolutionPaths();
        _planner->clear();

        setStartState(start);
        setGoalState(goal);

        _pdef->setStartAndGoalStates(*_start.get(), *_goal.get());
        _planner->setProblemDefinition(_pdef);
        _planner->setup();

        ob::PlannerTerminationConditionFn exactSolnPTC = std::bind(&GlobalPlanner::hasExactSolutionPTC, this);
        auto multiPTC = ob::plannerOrTerminationCondition(exactSolnPTC,
                                                      ob::timedPlannerTerminationCondition(_params.plan_time_lim));

        _soln_type = _planner->solve(multiPTC);


        if (_soln_type == ob::PlannerStatus::EXACT_SOLUTION || _soln_type == ob::PlannerStatus::APPROXIMATE_SOLUTION) 
        {// if success
            std::cout<<"[GlobalPlanner] Found solution!  ";
            

            auto t_f = std::chrono::steady_clock::now();
            
            
            _path.clear();
            
            _path_ptr = _pdef->getSolutionPath();

            og::PathGeometric *path = _path_ptr->as<og::PathGeometric>();

            std::cout<<"[cost: "<<_path_ptr->cost(_pdef->getOptimizationObjective())<<"]  ";
            std::cout<<"[dt_plan: "<<std::chrono::duration_cast<std::chrono::milliseconds>(t_f-t_i).count()<<"ms]  ";
            
            og::PathSimplifier path_simp(_si, _pdef->getGoal(), _pdef->getOptimizationObjective());
            
            path_simp.reduceVertices(*path, 10);
            
            t_i = std::chrono::steady_clock::now();
            path_simp.shortcutPath(*path, 100);
            t_f = std::chrono::steady_clock::now();
            std::cout<<"[dt_shorten: "<<std::chrono::duration_cast<std::chrono::milliseconds>(t_f-t_i).count()<<"ms]  ";

            
            t_i = std::chrono::steady_clock::now();
            path_simp.smoothBSpline(*path, 5);
            t_f = std::chrono::steady_clock::now();
            std::cout<<"[dt_smoothen: "<<std::chrono::duration_cast<std::chrono::milliseconds>(t_f-t_i).count()<<"ms]"<<std::endl;
            
            size_t num_states = path->getStateCount();
            _path.resize(num_states);

            auto states = path->getStates();
            for(size_t i =0; i< num_states; i++)
            {   
                _path[i] = states[i]->as<FlatMAVStateSpace::StateType>()->getXYZYaw();
            }            
        } 
        else
        {
            std::cout<<"[GlobalPlanner] Planning failed!\n";
        }
        // else if (ob::PlannerStatus::APPROXIMATE_SOLUTION) 
        // {// if sucess
        //     _path_approx.clear();
        //     std::cout<<"[GlobalPlanner] Found exact solution!\n";
            
        //     _path_ptr = _pdef->getSolutionPath();

        //     og::PathGeometric *path = _path_ptr->as<og::PathGeometric>();

        //     path_simp.smoothBSpline(*path, 5);
            
        //     size_t num_states = path->getStateCount();
            
        //     _path_approx.resize(num_states);

        //     for(size_t i =0; i< num_states; i++)
        //     {   
        //         _path_approx[i] = path->getStates()[i]->as<FlatMAVStateSpace::StateType>()->getXYZYaw();
        //     }            
        // } 
    }

    FlatMAVStateList GlobalPlanner::getPlannedPath(ob::PlannerStatus& type)
    {   
        type = _soln_type;
        return _path;
    }

    void GlobalPlanner::printPlanningData(std::string dir)
    {
        std::fstream fh,gh;

        // First write the planned path data
        fh.open(dir+"/path.csv", std::ios::out);

        if (!fh)
        {
            std::cout << "Path data file not created!\n";
        }
        else 
        {  
            for(auto pt : _path)
            {
                fh <<pt[0]<<" , " <<pt[1]<<" , " <<pt[2]<<" , " <<pt[3]<<"\n";
            }
            std::cout << "Path data file created successfully!\n";
            fh.close();
        }

        // Then write the graph data
        fh.open(dir+"/data.graphml", std::ios::out);
        if (!fh) {
            std::cout << "Graph data file not created!\n";
        }
        else 
        {
            ob::PlannerData data(_planner->getSpaceInformation());
            _planner->getPlannerData(data);
            data.printGraphML(fh);
            std::cout << "Graph data file created successfully!\n";
            fh.close();
        }

        // Then write the robot geometry data

        fh.open(dir+"/robot_model.csv", std::ios::out);
        if (!fh) {
            std::cout << "Robot data file not created!\n";
        }
        else 
        {   
            _mav_geometry.print(fh);    
            std::cout << "Robot data file created successfully!\n";
            fh.close();
        }
    }

    
}