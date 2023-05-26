#include <mav_planning/global_planner/global_planner_ros.h>

namespace mav_planning
{
    PlanManagerROS::PlanManagerROS(ros::NodeHandle nh_priv, ros::NodeHandle nh)
    {   
        _nh = nh;
        _nh_priv = nh_priv;

        _planner = std::make_shared<Planner>();
        _map_ptr = std::make_shared<Map>();
        _map_interface_ptr = std::shared_ptr<MapInterface>(_map_ptr);

        // std::shared_ptr<OctoMapInterface> octMapPtr = std::make_shared<OctoMapInterface>(_nh_priv, _nh);
        // _map_interface_ptr = std::shared_ptr<MapInterface>(octMapPtr);

        ros::spinOnce();

        // Subscribers
        // _goal_sub = _nh.subscribe("set_goal", 1, &PlanManagerROS::goalCB, this);
        // _start_sub = _nh.subscribe("set_start", 1, &PlanManagerROS::startCB, this);

        // Publishers
        _path_vis_pub = _nh.advertise<visualization_msgs::MarkerArray>("path_vis", 0, true);
        _approx_path_vis_pub = _nh.advertise<visualization_msgs::MarkerArray>("approx_path_vis", 0, true);

        _map_vis_pub = _nh.advertise<visualization_msgs::MarkerArray>("map_vis", 1, true);

        // Params
        _nh_priv.param<float>("planner/plan_time", _planner_params.plan_time_lim, 1.0);
        _nh_priv.param<bool>("planner/auto_range", _planner_params.use_auto_range, false);
        _nh_priv.param<float>("planner/range", _planner_params.planner_range, 5.0);
        _nh_priv.param<float>("planner/ground_clearance", _planner_params.svc_params.ground_clearance, 2.0);
        _nh_priv.param<float>("planner/virt_ceil_hgt", _planner_params.svc_params.virtual_ceiling, 2.0);
        _nh_priv.param<bool>("planner/optim/en_clear_obj", _planner_params.optim_params.en_clearance, false);
        _nh_priv.param<float>("planner/optim/k_length", _planner_params.optim_params.k_path_length, 1.0);
        _nh_priv.param<float>("planner/optim/k_clear", _planner_params.optim_params.k_clearance, 0.05);

        _mav_shape = Geometry(Geometry::GeometryType::SPHERE,
                              Point({0,0,0}),
                              Quaternion(Eigen::AngleAxisd(0.0, Point::UnitZ())),
                              std::vector<float>({3.0}));


        validateMapBounds();
        _map_ptr->setMapBounds(_map_bounds);
        _planner->registerMapInterface(_map_interface_ptr);

        processObstacles();
        
        _planner->setMAVShape(_mav_shape);
        _planner_params.type = Planner::PlannerType::InformedRRTStar;
        _planner->setupPlanner(_planner_params);
        
        visualizeMap();

        ros::Rate rate(5);
        size_t cnt = 0;

        while(ros::ok() && cnt < 10)
        {   
            ROS_INFO_STREAM("[PlanManagerROS] Replan Count: "<<cnt);
            _planner->planPath(FlatMAVState({_map_bounds.first.x()*0.99, _map_bounds.first.y()*0.99, 0.5*(_map_bounds.second.z()-_map_bounds.first.z()), 0.0}), 
                            FlatMAVState({_map_bounds.second.x()*0.99, _map_bounds.second.y()*0.99, 0.5*(_map_bounds.second.z()-_map_bounds.first.z()), 0.0}));
            // _planner->planPath(FlatMAVState({0,1.0,5.0, 0.0}), 
            //                 FlatMAVState({195,-71, 5.0, 0.0}));
            // _planner->planPath(FlatMAVState({-37.50, -37.50, 5.0, 0.0}), 
            //                 FlatMAVState({-37.50, 37.50, 5.0, 0.0}));
            visualizePlan();
            rate.sleep();
            cnt++;
        }

    }   

    void PlanManagerROS::validateMapBounds()
    {
        if(!_nh_priv.hasParam("map/bounds_min") && !_nh.hasParam("map/bounds_max"))
        {
            ROS_ERROR("[PlanManager] Map bounds missing! Quitting...");
        }
        else
        {
            std::vector<float> min_bounds, max_bounds;
            
            _nh_priv.getParam("map/bounds_min", min_bounds);
            _nh_priv.getParam("map/bounds_max", max_bounds);

            if(min_bounds.size()!=3 || max_bounds.size()!=3)
            {
                ROS_ERROR("[PlanManager] 3D Map bounds are required!");
            }
            else
            {
                _map_bounds.first << min_bounds[0], min_bounds[1], min_bounds[2];
                _map_bounds.second << max_bounds[0], max_bounds[1], max_bounds[2];
            }
        }

        _nh_priv.param<float>("plan_time_max", _planner_params.plan_time_lim);
        
    }

    void PlanManagerROS::processObstacles()
    {   
        bool read_obs_param;
        
        _nh_priv.param<bool>("map/get_from_param", read_obs_param, false);
        
        if(read_obs_param)
        {   
            ROS_INFO("[PlanManager] Reading obstacles from param server...");
            XmlRpc::XmlRpcValue obs;
            _nh_priv.getParam("map/obstacles", obs);
            
            if( obs.getType() != XmlRpc::XmlRpcValue::TypeArray ) {
                ROS_ERROR("Provided parameter map/obstacles is not a list!");
            }
            else
            {
                for(int i=0; i<obs.size(); i++)
                {
                    auto sublist = obs[i];

                    ROS_INFO_STREAM(sublist["type"]);
                    ROS_INFO_STREAM(sublist["shape"]);
                    ROS_INFO_STREAM(sublist["position"]);
                    ROS_INFO_STREAM(sublist["rotation"]);
                }
            }
        }
        else
        {   
            _nh_priv.param<int>("map/random_map/obs_count", _rand_map_params.count, 45);
            _nh_priv.param<float>("map/random_map/radius_min", _rand_map_params.min_radius, 10);
            _nh_priv.param<float>("map/random_map/radius_max", _rand_map_params.max_radius, 20);

            ROS_INFO("[PlanManager] Generating Random Map...");
            _map_ptr->generateRandomMap(_rand_map_params);
        }
    }

    void PlanManagerROS::visualizeMap()
    {
        MapInterface::MapType type = _map_ptr->getMapType();

        switch (type)
        {
            case MapInterface::MapType::CUSTOM3D:
            {
                auto obstacles = _map_ptr->getObstacles();

                visualization_msgs::Marker obs_marker;
                visualization_msgs::MarkerArray map_vis_msg;
                size_t count = 0;
                for(auto obs : obstacles)
                {
                    obs_marker = convert::toMarkerMsg(obs);
                    obs_marker.header.frame_id = "map";
                    obs_marker.header.stamp = ros::Time::now();
                    obs_marker.id = count;
                    map_vis_msg.markers.push_back(obs_marker);
                    count++;
                }

                _map_vis_pub.publish(map_vis_msg);
                break;
            }

            case MapInterface::MapType::OCTOMAP:
            {
                auto obstacles = _map_ptr->getObstacles();

                visualization_msgs::Marker obs_marker;
                visualization_msgs::MarkerArray map_vis_msg;
                size_t count = 0;
                for(auto obs : obstacles)
                {
                    obs_marker = convert::toMarkerMsg(obs);
                    obs_marker.header.frame_id = "map";
                    obs_marker.header.stamp = ros::Time::now();
                    obs_marker.id = count;
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
        visualization_msgs::MarkerArray plan_msg;
        visualization_msgs::Marker mav_msg;
        ob::PlannerStatus soln_type;
        
        auto path = _planner->getPlannedPath(soln_type);
        plan_msg = convert::toMarkerArrayMsg(path);

        
        if(soln_type==ob::PlannerStatus::APPROXIMATE_SOLUTION)
        {
            plan_msg.markers.back().color.r = 0.0;
            plan_msg.markers.back().color.g = 0.0;
            plan_msg.markers.back().color.b = 1.0;
            _approx_path_vis_pub.publish(plan_msg);
        }
        else
        {   
            plan_msg.markers.back().color.r = 1.0;
            plan_msg.markers.back().color.g = 0.0;
            plan_msg.markers.back().color.b = 0.0;
            size_t id = 0;
            for(auto pts : path)
            {   
                _mav_shape.setTranslation(Point{pts.x(), pts.y(), pts.z()});
                _mav_shape.setOrientation(convert::toEigenQuat(pts.w()));
                mav_msg = convert::toMarkerMsg(_mav_shape);
                mav_msg.color.r = 0.0;
                mav_msg.id = id;
                mav_msg.ns = "mav";

                plan_msg.markers.push_back(mav_msg);
                id++;
            }

            _path_vis_pub.publish(plan_msg);
        }

    }
    
}