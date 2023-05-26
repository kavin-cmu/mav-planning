#include <mav_planning/common/collision_geometry.h>

namespace mav_planning {
CollisionGeometry::CollisionGeometry(GeometryType type, Point position,
                                     Quaternion orientation,
                                     std::vector<float> shape) {

  _type = type;
  _shape = shape;
  _position = position;
  _orientation = orientation;

  switch (type) {
  case GeometryType::SPHERE: {
    _collision_obj = std::make_shared<fcl::CollisionObjectf>(
        std::shared_ptr<fcl::CollisionGeometryf>(new fcl::Spheref(shape[0])));
    break;
  }

  case GeometryType::BOX: {
    _collision_obj = std::make_shared<fcl::CollisionObjectf>(
        std::shared_ptr<fcl::CollisionGeometryf>(
            new fcl::Boxf(shape[0], shape[1], shape[2])));
    break;
  }

  case GeometryType::CYLINDER: {
    _collision_obj = std::make_shared<fcl::CollisionObjectf>(
        std::shared_ptr<fcl::CollisionGeometryf>(
            new fcl::Cylinderf(shape[0], shape[1])));
    break;
  }

  case GeometryType::ELLIPSOID: {
    _collision_obj = std::make_shared<fcl::CollisionObjectf>(
        std::shared_ptr<fcl::CollisionGeometryf>(
            new fcl::Ellipsoidf(shape[0], shape[1], shape[2])));
    break;
  }

  default: {
    std::cout << "[CollisionGeometry] Incompatible Obstacle type provided!";
    break;
  }
  }

  _collision_obj->setTranslation(position.cast<float>());
  _collision_obj->setQuatRotation(orientation.cast<float>());
}

CollisionGeometry::CollisionGeometry() {}

void CollisionGeometry::setTranslation(const Point &position) {
  _position = position;
  _collision_obj->setTranslation(position.cast<float>());
}

void CollisionGeometry::setOrientation(const Quaternion &orientation) {
  _orientation = orientation;
  _collision_obj->setQuatRotation(orientation.cast<float>());
}

Point CollisionGeometry::getTranslation() const { return _position; }

Quaternion CollisionGeometry::getOrientation() const { return _orientation; }

std::shared_ptr<fcl::CollisionObjectf>
CollisionGeometry::getCollisionObject() const {
  return _collision_obj;
}

CollisionGeometry::GeometryType CollisionGeometry::getType() const {
  return _type;
}

std::vector<float> CollisionGeometry::getShape() const { return _shape; }

void CollisionGeometry::print(std::ostream &out) const {
  std::string delim_lvl1 = " | ", delim_lvl2 = " , ";

  out << std::setprecision(3);

  for (size_t i = 0; i < 3; i++) {
    out << _position[i];

    if (i < 2) {
      out << delim_lvl2;
    }
  }

  out << delim_lvl1;
  out << _orientation.x() << delim_lvl2 << _orientation.y() << delim_lvl2
      << _orientation.z() << delim_lvl2 << _orientation.w();
  out << delim_lvl1;
  out << _type << delim_lvl1;

  for (size_t i = 0; i < _shape.size(); i++) {
    out << _shape[i];

    if (i < _shape.size() - 1) {
      out << delim_lvl2;
    }
  }
  out << "\n";
}

} // namespace mav_planning