#include <mav_planning/global_planner/global_planner_ros.h>

namespace mav_planning
{
    PlanManagerROS::PlanManagerROS(ros::NodeHandle nh_priv, ros::NodeHandle nh)
    {   
        _nh = nh;
        _nh_priv = nh_priv;

        int space_type;
        _nh_priv.param<int>("planner/state_space", space_type, 0);

        _planner = std::make_shared<Planner>(StateSpaceType(space_type));
        // _map_ptr = std::make_shared<Map>();
        // _map_interface_ptr = std::shared_ptr<MapInterface>(_map_ptr);

        _oct_map_ptr = std::make_shared<OctoMapInterface>(_nh_priv, _nh);
        _map_interface_ptr = std::shared_ptr<MapInterface>(_oct_map_ptr);

        // Timer loops
        _controlLoop = _nh.createTimer(ros::Duration(0.02), &PlanManagerROS::controllerCB, this);
        _planLoop = _nh.createTimer(ros::Duration(2.0), &PlanManagerROS::plannerCB, this);

        ros::spinOnce();

        // Subscribers
        _goal_sub = _nh.subscribe("/move_base_simple/goal", 1, &PlanManagerROS::goalCB, this);
        _start_sub = _nh.subscribe("/initialpose", 1, &PlanManagerROS::startCB, this);
        _odom_sub = _nh.subscribe("/drone_0_visual_slam/odom", 1, &PlanManagerROS::odomCB, this);

        // Publishers
        _path_vis_pub = _nh.advertise<visualization_msgs::MarkerArray>("path_vis", 0, true);
        _approx_path_vis_pub = _nh.advertise<visualization_msgs::MarkerArray>("approx_path_vis", 0, true);
        _map_vis_pub = _nh.advertise<visualization_msgs::MarkerArray>("map_vis", 1, true);
        _cmd_pub = _nh.advertise<quadrotor_msgs::PositionCommand>("/drone_0_planning/pos_cmd", 1, true);

        // Params
        int planner_type;
        _nh_priv.param<int>("planner/type", planner_type, 3);
        _planner_params.type = Planner::Type(planner_type);

        XmlRpc::XmlRpcValue mav_shape_param;
        _nh_priv.getParam("planner/mav_shape", mav_shape_param);
        _nh_priv.param<double>("planner/state_interp_res", _planner_params.interp_resolution, 0.50);
        _nh_priv.param<double>("planner/plan_time", _planner_params.plan_time_lim, 1.0);
        _nh_priv.param<bool>("planner/auto_range", _planner_params.use_auto_range, false);
        _nh_priv.param<bool>("planner/smoothen", _planner_params.smoothen, false);
        _nh_priv.param<double>("planner/range", _planner_params.planner_range, 5.0);
        _nh_priv.param<bool>("planner/use_full_planning_time", _planner_params.use_full_plan_time, true);
        _nh_priv.param<bool>("planner/wait_for_exact_soln", _planner_params.wait_for_exact_soln, false);
        _nh_priv.param<double>("planner/ground_clearance", _planner_params.svc_params.ground_clearance, 2.0);
        _nh_priv.param<double>("planner/virt_ceil_hgt", _planner_params.svc_params.virtual_ceiling, 2.0);
        _nh_priv.param<bool>("planner/optim/en_clear_obj", _planner_params.optim_params.en_clearance, false);
        _nh_priv.param<double>("planner/optim/k_length", _planner_params.optim_params.k_path_length, 1.0);
        _nh_priv.param<double>("planner/optim/k_clear", _planner_params.optim_params.k_clearance, 0.05);
        _nh_priv.param<double>("planner/goal_z", _goal_z, 2.0);
        _nh_priv.param<double>("planner/start_z", _start_z, 2.0);


        // std::vector<double> start(4);
        // _nh_priv.getParam("planner/start_state", start);
        // _start = FlatMAVState({start[0], start[1], start[2], start[3]});

        _nh_priv.param<int>("map/random_map/obs_count", _rand_map_params.count, 45);
        _nh_priv.param<double>("map/random_map/radius_min", _rand_map_params.min_radius, 10);
        _nh_priv.param<double>("map/random_map/radius_max", _rand_map_params.max_radius, 20);
        

        // validateMapBounds();
        // _map_ptr->setMapBounds(_map_bounds);

        WARN(_name, "Waiting for Map!");

        while(!_oct_map_ptr->hasInit && ros::ok())
        {
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }

        INFO(_name, "Map received!");

        _planner->registerMapInterface(_map_interface_ptr);

        // INFO(_name, "Map received! 2");


        if(convert::toCollisionGeometry(mav_shape_param, _mav_shape))
        {
            _planner->setMAVShape(_mav_shape);
        }
        else
        {
            ERROR(_name,"Incorrect geometry format for MAV shape from rosparam!");
        }

        _planner->setupPlanner(_planner_params);

        // processObstacles();
        // visualizeMap();
        // visualizeMap();
    } 

    void PlanManagerROS::odomCB(const nav_msgs::Odometry::ConstPtr & odom)
    {
        if(!_flag_recvd_odom)
        {
            _flag_recvd_odom = true;
        }

        _curr_pose = convert::toSE3State(odom->pose.pose);
        _curr_vel[0] = odom->twist.twist.linear.x;
        _curr_vel[1] = odom->twist.twist.linear.y;
        _curr_vel[2] = odom->twist.twist.linear.z;
    }

    // void PlanManagerROS::runPlanner(const SE3State& start, const SE3State& goal)
    // {
    //     ros::Rate rate(0.50);
    //     size_t cnt = 0;
    //     Point edt_size = {40,40,10};

    //     _planner->setupPlanner(_planner_params);

    //     while(ros::ok() && (_curr_pose.position-_goal.position).norm()>1.0)
    //     {   
    //         ob::PlannerStatus soln_type;
    //         INFO(_name,"[Replan %ld]", cnt+1);
    //         _planner->planPath(_curr_pose, goal);
    //         // if(soln_type == ob::PlannerStatus::EXACT_SOLUTION)
    //         // {
    //         _have_path = true;
    //         _curr_path.clear();
    //         _curr_path.resize(_planner->getPlannedPath(soln_type).size());
    //         _curr_path = _planner->getPlannedPath(soln_type);
    //         _pt_idx = 0;
    //         visualizePlan();
    //         // }
    //         // ros::spinOnce();
    //         rate.sleep();
    //         cnt++;
    //     }

    //     INFO(_name,"Planning done!");
    // } 

    void PlanManagerROS::plannerCB(const ros::TimerEvent & event) 
    {

        if(_have_goal && _flag_recvd_odom && !_reached_goal)
        {   
            ob::PlannerStatus soln_type;
            INFO(_name,"[Replan %ld]", _replan_cnt+1);
            _planner->planPath(_curr_pose, _goal);
            _have_path = true;
            _curr_path.clear();
            _curr_path.resize(_planner->getPlannedPath(soln_type).size());
            _curr_path = _planner->getPlannedPath(soln_type);
            _pt_idx = 0;
            visualizePlan();
            _replan_cnt++;
        }
        else if(_reached_goal)
        {
            _have_goal = false;
            INFO(_name,"Planning done!");
        }

        if(_new_goal)
        {
            _new_goal = false;
            _replan_cnt = 0;
        }

    }

    void PlanManagerROS::controllerCB(const ros::TimerEvent & event)
    {
        if(_have_path)
        {   
            
            _pt_idx = std::min(_curr_path.size()-1, _pt_idx);
            quadrotor_msgs::PositionCommand cmd;
            cmd.header.frame_id = "world";
            cmd.header.stamp = ros::Time::now();
            cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
            cmd.trajectory_id = _pt_idx;
            cmd.position.x = _curr_path[_pt_idx].position.x();
            cmd.position.y = _curr_path[_pt_idx].position.y();       
            cmd.position.z = _curr_path[_pt_idx].position.z();       
            _pt_idx++;
            _cmd_pub.publish(cmd);

            if(!_reached_goal && (_goal.position-_curr_pose.position).norm()<1.0)
            {
                _reached_goal = true;
            }
        }
    }


    void PlanManagerROS::validateMapBounds()
    {
        if(!_nh_priv.hasParam("map/bounds_min") && !_nh.hasParam("map/bounds_max"))
        {
            ERROR(_name, "Map bounds missing! Quitting...");
        }
        else
        {
            std::vector<double> min_bounds, max_bounds;
            
            _nh_priv.getParam("map/bounds_min", min_bounds);
            _nh_priv.getParam("map/bounds_max", max_bounds);

            if(min_bounds.size()!=3 || max_bounds.size()!=3)
            {
                ERROR(_name,"3D Map bounds are required!");
            }
            else
            {
                _map_bounds.first << min_bounds[0], min_bounds[1], min_bounds[2];
                _map_bounds.second << max_bounds[0], max_bounds[1], max_bounds[2];
            }
        }

        _nh_priv.param<double>("plan_time_max", _planner_params.plan_time_lim);
        
    }

    void PlanManagerROS::processObstacles()
    {   
        bool read_obs_param;
        
        _nh_priv.param<bool>("map/get_from_param", read_obs_param, false);
        
        if(read_obs_param)
        {   
            INFO(_name, "Reading obstacles from param server...");
            XmlRpc::XmlRpcValue obs_list;
            _nh_priv.getParam("map/obstacles", obs_list);
            
            if( obs_list.getType() != XmlRpc::XmlRpcValue::TypeArray ) {
                ERROR(_name,"Provided parameter map/obstacles is not a list!");
            }
            else
            {
                for(int i=0; i<obs_list.size(); i++)
                {   
                    Geometry shape;
                    if(convert::toCollisionGeometry(obs_list[i], shape))
                    {
                        _map_ptr->addObstacle(shape);
                    }
                }
            }
        }
        else
        {  
            INFO(_name,"Generating Random Map...");
            _map_ptr->generateRandomMap(_rand_map_params);
        }
    }

    void PlanManagerROS::visualizeMap()
    {
		MapInterface::Type type = _map_ptr->getType();

		switch (type)
		{
			case MapInterface::Type::CUSTOM3D:
			{
					auto obstacles = _map_ptr->getObstacles();

					visualization_msgs::Marker obs_marker;
					visualization_msgs::MarkerArray map_vis_msg;
					size_t count = 0;
					for(auto obs : obstacles)
					{
						obs_marker = vis_utils::getDisplayMarker(obs, vis_utils::Color::GREY, count, "map");
						map_vis_msg.markers.push_back(obs_marker);
						count++;
					}

					_map_vis_pub.publish(map_vis_msg);
					break;
			}
					
			default:
					break;
		}
    
    }

    void PlanManagerROS::visualizePlan()
    {
		INFO(_name, "Visualizing plan..");
        visualization_msgs::MarkerArray comb_msg;
		visualization_msgs::Marker path_msg, mav_msg, goal_msg, start_msg, obb_msg, aabb_msg;
		ob::PlannerStatus soln_type;

		size_t id = 0;
		for(auto pt : _curr_path)
		{   
			_mav_shape.setTranslation(pt.position);
			_mav_shape.setOrientation(pt.rotation);
			mav_msg = vis_utils::getDisplayMarker(_mav_shape, vis_utils::Color::GREY, id, "mav");
			// obb_msg = vis_utils::getDisplayMarker(_mav_shape.getOBB(), vis_utils::Color::TRANSLUCENT_DARK, id, "obb");
			// aabb_msg = vis_utils::getDisplayMarker(_mav_shape.getAABB(), vis_utils::Color::TRANSLUCENT_RED, id, "aabb");
			comb_msg.markers.push_back(mav_msg);
			// comb_msg.markers.push_back(obb_msg);
			// comb_msg.markers.push_back(aabb_msg);

			id++;
		}

		auto goal = _planner->getGoalState();
		goal_msg = vis_utils::getDisplayMarker(goal.position, 
                                                vis_utils::Color::RED,
                                                1.0, 0, "goal");

		comb_msg.markers.push_back(goal_msg);

        auto start = _planner->getStartState();
		start_msg = vis_utils::getDisplayMarker(start.position, 
                                                vis_utils::Color::CYAN,
                                                1.0, 0, "start");

		comb_msg.markers.push_back(start_msg);
		
		if(soln_type==ob::PlannerStatus::APPROXIMATE_SOLUTION)
		{
			path_msg = vis_utils::getDisplayMarker(_curr_path, vis_utils::Color::ORANGE, 0.30f, 0u, "path");
			comb_msg.markers.push_back(path_msg);
			_approx_path_vis_pub.publish(comb_msg);
		}
		else
		{   
			path_msg = vis_utils::getDisplayMarker(_curr_path, vis_utils::Color::GREEN, 0.30f, 0u, "path");
			comb_msg.markers.push_back(path_msg);
			_path_vis_pub.publish(comb_msg);
		}

    }

    void PlanManagerROS::goalCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        SE3State goal = convert::toSE3State(msg->pose);

        if(_goal.position!=goal.position)
        {
            _goal = goal;
            _goal.position.z() = _goal_z;
            _goal.rotation.setIdentity();
            _new_goal = true;
            _have_goal= true;
            _reached_goal = false;
        }
    }

    void PlanManagerROS::startCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
    {
        _start = convert::toSE3State(msg->pose.pose);
        _start.position.z() = _start_z;
        _start.rotation.setIdentity();
    }
    
}