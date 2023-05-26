#include <mav_planning/global_planner/map_interfaces/octomap_interface.h>

namespace mav_planning
{   
    bool OctoMapInterface::checkCollision(const CollisionGeometry& mav_shape)
    {
        fcl::CollisionRequest<float> colRequest(1,false,1,false);
        fcl::CollisionResult<float> colResult;

        return fcl::collide(mav_shape.getCollisionObject().get(), _map_col_obj.get(), colRequest, colResult);
    }

    float OctoMapInterface::getMinClearance(const CollisionGeometry& mav_shape)
    {
        auto pos = mav_shape.getTranslation();

        octomap::point3d obs_pos, query_pos(pos[0], pos[1], pos[2]);

        float min_dist;

        _edt_map_ptr->getDistanceAndClosestObstacle(query_pos, min_dist, obs_pos);

        return min_dist;

    }

    // bool OctoMapInterface::openMapFile(const std::string& filepath)
    // {   
    //     _tree_depth = _octree->getTreeDepth();

    //     ROS_INFO_STREAM(_tree_depth);

    //     _octree->getMetricMin(_bounds.first[0], _bounds.first[1], _bounds.first[2]);
    //     _octree->getMetricMax(_bounds.second[0], _bounds.second[1], _bounds.second[2]);
    //     _has_bounds = true;

    //     octomap::point3d p_min(_bounds.first[0], _bounds.first[1], _bounds.first[2]);
    //     octomap::point3d p_max(_bounds.second[0], _bounds.second[1], _bounds.second[2]);

    //     ROS_INFO("[OctoMapInterface] Min Bounds: [%.2f, %.2f, %.2f]",_bounds.first[0], _bounds.first[1], _bounds.first[2]);
    //     ROS_INFO("[OctoMapInterface] Max Bounds: [%.2f, %.2f, %.2f]",_bounds.second[0], _bounds.second[1], _bounds.second[2]);

    //     _map_col_obj = std::make_shared<fcl::CollisionObjectf>(std::shared_ptr<fcl::CollisionGeometryf>(new fcl::OcTreef(_octree)));
    //     _edt_map_ptr = std::make_shared<DynamicEDTOctomap>(3.0, _octree.get(), p_min, p_max, false);

    //     return true;
    // }

    void OctoMapInterface::updateEDTMap()
    {
        _edt_map_ptr->update();
    }
}
