#include <mav_planning/global_planner/map_interfaces/custom_map_interface.h>


namespace mav_planning
{
    void Custom3DMap::addObstacle(const CollisionGeometry& obs)
    {
        if(obs.getType()==CollisionGeometry::Type::INVALID)
        {
            ERROR(_name, "Invalid obstacle! Skipping...");
        }
        _obstacles.push_back(obs);
    }

    void Custom3DMap::clearMap()
    {
        _obstacles.clear();
        _has_bounds = false;
    }

    size_t Custom3DMap::getNumObstacles()
    {
        return _obstacles.size();
    }

    double Custom3DMap::getMinClearance(const SE3State& state)
    {
        
        updateMAVState(state);
        
        fcl::DistanceRequest<double> requestType;
        fcl::DistanceResult<double> distanceResult;

        double min_dist = std::numeric_limits<double>::max();
                
        for(CollisionGeometry obs : _obstacles)
        {   
            fcl::distance(_mav_shape.getCollisionObject().get(), obs.getCollisionObject().get(), requestType, distanceResult);
            min_dist = std::min(min_dist, distanceResult.min_distance);
        }

        return min_dist;
    }

    bool Custom3DMap::checkCollision(const SE3State& state)
    {   
        updateMAVState(state);

        fcl::CollisionRequest<double> colRequest(1,false,1,false);
        fcl::CollisionResult<double> colResult;

        bool has_collided = false;

        for(CollisionGeometry obs : _obstacles)
        {  
            fcl::collide(_mav_shape.getCollisionObject().get(), obs.getCollisionObject().get(), colRequest, colResult);
            has_collided = colResult.isCollision();
            
            if(has_collided)
            {   
                break;
            }
        }

        return has_collided;
    }

    void Custom3DMap::setMapBounds(const std::pair<Point, Point>& bounds)
    {
        _bounds = bounds;
        _has_bounds = true;
    }

    std::pair<Point, Point> Custom3DMap::getMapBounds()
    {
        return _bounds;
    }

    std::vector<CollisionGeometry> Custom3DMap::getObstacles()
    {
        return _obstacles;
    }

    void Custom3DMap::printMap(std::string dir)
    {      
        std::fstream fh;

        fh.open(dir+"/map.csv", std::ios::out);

        if (!fh)
        {
            ERROR(_name,"Map data file not created!");
        }
        else
        {   
            fh << getMapBounds().first[0] << " , " << getMapBounds().first[1] << " , "<< getMapBounds().first[2] << "\n";
            fh << getMapBounds().second[0] << " , " << getMapBounds().second[1] << " , "<< getMapBounds().second[2] << "\n";
            for (CollisionGeometry obs : _obstacles)
            {   
                obs.print(fh);
            }
            INFO(_name, "Map data file created succesfully!");
            fh.close();
        }
    }

    void Custom3DMap::generateRandomMap(const RandomMapParams& params)
    {   
        std::random_device rd;

        std::uniform_real_distribution<double> unif_x(_bounds.first[0], _bounds.second[0]);
        std::uniform_real_distribution<double> unif_y(_bounds.first[1], _bounds.second[1]);
        std::uniform_real_distribution<double> unif_z(_bounds.first[2], _bounds.second[2]);
        std::uniform_real_distribution<double> unif_r(params.min_radius, params.max_radius); 
        
        std::default_random_engine re(rd());

        double z_clearance = _bounds.second[2]-_bounds.first[2];

        for(size_t i=0; i< params.count/3; i++)
        {
            addObstacle(CollisionGeometry(CollisionGeometry::Type::CYLINDER,
                        Point({unif_x(re),unif_y(re), 0.5*z_clearance}),
                        Quaternion(Eigen::AngleAxisd(0.0, Point::UnitZ())),
                        std::vector<double>({unif_r(re), z_clearance})));
        }
        for(size_t i=0; i< params.count/3; i++)
        {
            addObstacle(CollisionGeometry(CollisionGeometry::Type::BOX,
                        Point({unif_x(re),unif_y(re), 0.5*z_clearance}),
                        Quaternion(Eigen::AngleAxisd(0.0, Point::UnitZ())),
                        std::vector<double>({unif_r(re), unif_r(re), z_clearance})));
        }
        for(size_t i=0; i< params.count/3; i++)
        {   
            addObstacle(CollisionGeometry(CollisionGeometry::Type::SPHERE,
                        Point({unif_x(re), unif_y(re), unif_z(re)}),
                        Quaternion(Eigen::AngleAxisd(0.0, Point::UnitZ())),
                        std::vector<double>({unif_r(re)})));
        }
    }

}
