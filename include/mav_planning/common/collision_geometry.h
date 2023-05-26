#ifndef MAP_INTERFACES_COLLISION_INTERFACE_COLLISION_GEOMETRY
#define MAP_INTERFACES_COLLISION_INTERFACE_COLLISION_GEOMETRY

#include <mav_planning/common/common.h>

#include <fcl/fcl.h>
#include <fcl/config.h>
#include <fcl/math/geometry.h>
#include <fcl/geometry/collision_geometry.h>


namespace mav_planning
{
    class CollisionGeometry
    {
        public:
            enum GeometryType{SPHERE, BOX, CYLINDER, ELLIPSOID};

            CollisionGeometry(GeometryType type, Point position,
                              Quaternion orientation, std::vector<float> shape);
            
            CollisionGeometry();
            
            void setTranslation(const Point& position);
            void setOrientation(const Quaternion& orientation);
            Point getTranslation() const;
            Quaternion getOrientation() const;
            GeometryType getType() const;
            std::vector<float> getShape() const;
            std::shared_ptr<fcl::CollisionObjectf> getCollisionObject() const;
            void print(std::ostream& os) const;

        private:
            GeometryType _type;
            std::vector<float> _shape;
            std::shared_ptr<fcl::CollisionObjectf> _collision_obj;
            Point _position;
            Quaternion _orientation;
    };

}


#endif /* MAP_INTERFACES_COLLISION_INTERFACE_COLLISION_GEOMETRY */
