#include <mav_planning/global_planner/map_interfaces/octomap_interface.h>

namespace mav_planning
{   
    bool OctoMapInterface::checkCollision(const SE3State& state)
    {
        
        updateMAVState(state);
        
        Point pos = state.position;

        // if(!_octree->search(pos[0], pos[1], pos[2]))
        // {
        //     return true;
        // };

        fcl::CollisionRequest<double> colRequest(5,false,1,false, true);
        fcl::CollisionResult<double> colResult;

        fcl::collide(_mav_shape.getCollisionObject().get(), _map_col_obj.get(), colRequest, colResult);

        return colResult.isCollision();
    }

    double OctoMapInterface::getMinClearance(const SE3State& state)
    {
        updateMAVState(state);
        
        auto pos = state.position;

        octomap::point3d query_pos(pos[0], pos[1], pos[2]);

        float min_dist = _edt_map_ptr->getDistance(query_pos);

        return min_dist;

    }

    void OctoMapInterface::mapCB(const octomap_msgs::Octomap::ConstPtr& msg)
    {   
        if(!_map_locked)
        {
            _octree = std::shared_ptr<octomap::OcTree>(dynamic_cast<octomap::OcTree*>(octomap_msgs::binaryMsgToMap(*msg)));
            _tree_depth = _octree->getTreeDepth();

            // INFO_SPECIAL(_name, "Tree Depth: %d", _tree_depth);
            // INFO_SPECIAL(_name, "Map Resolution: %.2lf", _octree->getResolution());

            _octree->getMetricMin(_bounds.first[0], _bounds.first[1], _bounds.first[2]);
            _octree->getMetricMax(_bounds.second[0], _bounds.second[1], _bounds.second[2]);
            _has_bounds = true;

            // INFO_SPECIAL(_name, "Min Bounds: [%.2f, %.2f, %.2f]",_bounds.first[0], _bounds.first[1], _bounds.first[2]);
            // INFO_SPECIAL(_name, "Max Bounds: [%.2f, %.2f, %.2f]",_bounds.second[0], _bounds.second[1], _bounds.second[2]);

            _map_col_obj = std::make_shared<fcl::CollisionObjectd>(std::shared_ptr<fcl::CollisionGeometryd>(new fcl::OcTreed(_octree)));   

            if(!hasInit)
            {
                hasInit = true;
            }
        }
        
    }

    void OctoMapInterface::displayEDTMap(double max_dist, octomap::point3d& min_p, octomap::point3d& max_p)
    {   
        auto res = _octree->getResolution();

        visualization_msgs::Marker msg;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "map";
        msg.action = visualization_msgs::Marker::ADD;
        msg.type = visualization_msgs::Marker::CUBE_LIST;
        msg.scale.x = msg.scale.y = msg.scale.z = 0.5*res;
        msg.pose.orientation.w = 1;
        msg.id = 0;
        msg.ns = "sdf_map";
        msg.color.a = 1.0;

        for(float x = min_p(0); x<max_p(0); x+=2*res)
        {
            for(float y = min_p(1); y<max_p(1); y+=2*res)
            {   
                for(auto z = min_p(2); z<max_p(2); z+=2*res)
                {
                    float dist = _edt_map_ptr->getDistance(octomap::point3d(x,y,z));
                    geometry_msgs::Point p;
                    std_msgs::ColorRGBA color;

                    color.r = 1-dist/max_dist;
                    color.g = 0.0;
                    color.b = 0.0;
                    color.a = 0.10;

                    p.x = x;
                    p.y = y;
                    p.z = z;
                    msg.points.push_back(p);
                    msg.colors.push_back(color);                    
                }
            }
        }

        _sdf_pub.publish(msg);
    }

    void OctoMapInterface::updateEDTMap(float max_dist, const Point& p_min, const Point& p_max)
    {   
        auto t_i = std::chrono::steady_clock::now();
        _edt_bounds.first = p_min;
        _edt_bounds.second = p_max;
        octomap::point3d min_p(p_min[0], p_min[1], p_min[2]);
        octomap::point3d max_p(p_max[0], p_max[1], p_max[2]);
        _edt_map_ptr = std::make_shared<DynamicEDTOctomap>(max_dist, _octree.get(), min_p, max_p, false);
        _edt_map_ptr->update();
        auto t_f = std::chrono::steady_clock::now();
        auto dt_update = std::chrono::duration_cast<std::chrono::milliseconds>(t_f - t_i).count();
        INFO_SPECIAL(_name, "EDT Map Update took: %ldms", dt_update);
        displayEDTMap(max_dist, min_p, max_p);
    }
}
