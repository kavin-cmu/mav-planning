#ifndef GLOBAL_PLANNER_MAP_INTERFACES_CUSTOM_MAP_INTERFACE
#define GLOBAL_PLANNER_MAP_INTERFACES_CUSTOM_MAP_INTERFACE

#include <mav_planning/global_planner/collision_checking/planner_map_interface.h>

namespace mav_planning
{

    class Custom3DMap: public PlannerMapInterface
    {   
        public:
            Custom3DMap(void):PlannerMapInterface(Type::CUSTOM3D){};

            ~Custom3DMap(void){};

            struct RandomMapParams{
                int count;
                double min_radius, max_radius;
            };

            void setMapBounds(const std::pair<Point, Point>& bounds);
            std::pair<Point, Point> getMapBounds();
            
            void addObstacle(const CollisionGeometry& obstacle);
            std::vector<CollisionGeometry> getObstacles();
            size_t getNumObstacles();
            void clearMap();
            
            void generateRandomMap(const RandomMapParams& params);

            // Overriden methods from PlannerMapInterface
            bool checkCollision(const SE3State& state);
            double getMinClearance(const SE3State& state);

            // void readFromFile(std::string filepath);
            void printMap(std::string dir);
        
        private:
            std::string _name = "CustomMapInterface";
            std::vector<CollisionGeometry> _obstacles;
    };
}



#endif /* GLOBAL_PLANNER_MAP_INTERFACES_CUSTOM_MAP_INTERFACE */
