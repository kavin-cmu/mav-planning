#ifndef MAV_PLANNING_COMMON_COLLISION_GEOMETRY
#define MAV_PLANNING_COMMON_COLLISION_GEOMETRY

#include <mav_planning/common/common.h>
#include <mav_planning/common/logging.h>


#include <fcl/fcl.h>
#include <fcl/config.h>
#include <fcl/math/geometry.h>
#include <fcl/geometry/collision_geometry.h>


namespace mav_planning
{   
    struct AABBox
    {
        Point min;
        Point max;
    };


    class CollisionGeometry
    {
        public:
            enum Type{SPHERE, BOX, CYLINDER, ELLIPSOID, INVALID};

            CollisionGeometry(Type type, Point position,
                              Quaternion orientation, std::vector<double> shape);
            
            CollisionGeometry();
            
            void setTranslation(const Point& position);
            void setOrientation(const Quaternion& orientation);
            Point getTranslation() const;
            Quaternion getOrientation() const;
            Type getType() const;
            std::vector<double> getShape() const;
            std::shared_ptr<fcl::CollisionObjectd> getCollisionObject() const;
            std::vector<Point> getAABBoxVerts(fcl::AABBd& aabb);
            AABBox getAABB();
            fcl::OBBd getOBB();
            void print(std::ostream& os) const;

        private:
            Type _type = Type::INVALID;
            std::vector<double> _shape;
            std::shared_ptr<fcl::CollisionObjectd> _collision_obj = NULL;
            Point _position;
            Quaternion _orientation;
    };

}


#endif /* MAV_PLANNING_COMMON_COLLISION_GEOMETRY */
