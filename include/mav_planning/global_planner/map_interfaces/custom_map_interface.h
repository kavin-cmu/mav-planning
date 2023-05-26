#ifndef GLOBAL_PLANNER_MAP_INTERFACES_CUSTOM_MAP_INTERFACE
#define GLOBAL_PLANNER_MAP_INTERFACES_CUSTOM_MAP_INTERFACE

#include <mav_planning/global_planner/collision_checking/planner_map_interface.h>

namespace mav_planning
{

    class Custom3DMap: public PlannerMapInterface
    {   
        public:
            Custom3DMap(void):PlannerMapInterface(MapType::CUSTOM3D){};

            ~Custom3DMap(void){};

            struct RandomMapParams{
                int count;
                float min_radius, max_radius;
            };

            void setMapBounds(const std::pair<Point, Point>& bounds);
            std::pair<Point, Point> getMapBounds();
            
            void addObstacle(const CollisionGeometry& obstacle);
            void generateRandomMap(const RandomMapParams& params);
            size_t getNumObstacles();
            std::vector<CollisionGeometry> getObstacles();
            void clearMap();

            // Overriden methods from PlannerMapInterface
            bool checkCollision(const CollisionGeometry& shape);
            float getMinClearance(const CollisionGeometry& shape);

            // void readFromFile(std::string filepath);
            void printMap(std::string dir);
        
        private:
            std::vector<CollisionGeometry> _obstacles;

    };
}



#endif /* GLOBAL_PLANNER_MAP_INTERFACES_CUSTOM_MAP_INTERFACE */
