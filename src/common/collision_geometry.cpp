#include <mav_planning/common/collision_geometry.h>

namespace mav_planning {
CollisionGeometry::CollisionGeometry(Type type, Point position,
                                     Quaternion orientation,
                                     std::vector<double> shape) {

  _type = type;
  _shape = shape;
  _position = position;
  _orientation = orientation;

  switch (type) {
  case Type::SPHERE: {
    _collision_obj = std::make_shared<fcl::CollisionObjectd>(
        std::shared_ptr<fcl::CollisionGeometryd>(new fcl::Sphered(shape[0])));
    break;
  }

  case Type::BOX: {
    _collision_obj = std::make_shared<fcl::CollisionObjectd>(
        std::shared_ptr<fcl::CollisionGeometryd>(
            new fcl::Boxd(shape[0], shape[1], shape[2])));
    break;
  }

  case Type::CYLINDER: {
    _collision_obj = std::make_shared<fcl::CollisionObjectd>(
        std::shared_ptr<fcl::CollisionGeometryd>(
            new fcl::Cylinderd(shape[0], shape[1])));
    break;
  }

  case Type::ELLIPSOID: {
    _collision_obj = std::make_shared<fcl::CollisionObjectd>(
        std::shared_ptr<fcl::CollisionGeometryd>(
            new fcl::Ellipsoidd(shape[0], shape[1], shape[2])));
    break;
  }

  default: {
    ERROR("CollisionGeometry","Incompatible Obstacle type provided!");
    break;
  }
  }
  if(!_collision_obj)
  {
    _collision_obj->setTranslation(position.cast<double>());
    _collision_obj->setQuatRotation(orientation.cast<double>());
  }
}

CollisionGeometry::CollisionGeometry() {}

// CollisionGeometry::CollisionGeometry(CollisionGeometry& other)
// {
//   _type = other.getType();
//   _shape = other.getShape();
//   _position = other.getTranslation();
//   _orientation = other.getOrientation();
//   _collision_obj = other.getCollisionObject();
  
// }

void CollisionGeometry::setTranslation(const Point &position) {
  _position = position;
  _collision_obj->setTranslation(position.cast<double>());
}

void CollisionGeometry::setOrientation(const Quaternion &orientation) {
  _orientation = orientation;
  _collision_obj->setQuatRotation(orientation.cast<double>());
}

Point CollisionGeometry::getTranslation() const { return _position; }

Quaternion CollisionGeometry::getOrientation() const { return _orientation; }

std::shared_ptr<fcl::CollisionObjectd> CollisionGeometry::getCollisionObject() const 
{
  return _collision_obj;
}

CollisionGeometry::Type CollisionGeometry::getType() const 
{
  return _type;
}

std::vector<double> CollisionGeometry::getShape() const { return _shape; }

std::vector<Point> CollisionGeometry::getAABBoxVerts(fcl::AABBd& aabb)
{
  Eigen::Vector3d extent = {0.5*aabb.width(), 0.5*aabb.depth(), 0.5*aabb.height()};
  std::vector<Point> verts(8);
  
  verts[0] = { - extent[0], + extent[1], + extent[2] };
  verts[1] = { + extent[0], + extent[1], + extent[2] };
  verts[2] = { - extent[0], - extent[1], + extent[2] };
  verts[3] = { + extent[0], - extent[1], + extent[2] };
  verts[4] = { - extent[0], + extent[1], - extent[2] };
  verts[5] = { + extent[0], + extent[1], - extent[2] };
  verts[6] = { - extent[0], - extent[1], - extent[2] };
  verts[7] = { + extent[0], - extent[1], - extent[2] };

  
  return verts;

}

AABBox CollisionGeometry::getAABB()
{ 
  // Get local AABB
  fcl::AABBd local_aabb = _collision_obj->getCollisionGeometry()->aabb_local;
  
  // Obtain its attributes
  Eigen::Vector3d extent = {0.5*local_aabb.width(), 0.5*local_aabb.depth(), 0.5*local_aabb.height()};
  fcl::Matrix3d rot = _collision_obj->getRotation().matrix();
  Point trans = _collision_obj->getTranslation();

  std::vector<Point> verts_local(8), verts(8);
  
  // calculate its verts
  verts_local = getAABBoxVerts(local_aabb);

  Eigen::Vector3d max_val = -Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
  Eigen::Vector3d min_val = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());


  for (int i=0; i<8; i++)
  { 
    // transform its verts to global frame
    verts[i] = trans + rot*verts_local[i];
    
    // keep track of max and min coordinates of its verts
    for(int j=0; j<3; j++)
    {
      if(min_val[j]>verts[i][j])
      {
        min_val[j] = verts[i][j];
      }
      if(max_val[j]<verts[i][j])
      {
        max_val[j] = verts[i][j];
      }
    }
  }

  // store min-max values
  AABBox bbx;
  bbx.min = min_val;
  bbx.max = max_val;

  return bbx;

}

fcl::OBBd CollisionGeometry::getOBB()
{
  fcl::AABBd aabb = _collision_obj->getCollisionGeometry()->aabb_local;
  
  Eigen::Vector3d extent = {0.5*aabb.width(), 0.5*aabb.depth(), 0.5*aabb.height()};
  fcl::Matrix3d rot = _collision_obj->getRotation();
  fcl::Vector3d trans = 0.0*aabb.center()+_collision_obj->getTranslation();

  return fcl::OBBd(rot, trans, extent);
}

void CollisionGeometry::print(std::ostream &out) const 
{
  std::string attr_delim = " | ", value_delim = " , ";

  out << std::setprecision(3);

  for (size_t i = 0; i < 3; i++) 
  {
    out << _position[i];

    if (i < 2) 
    {
      out << value_delim;
    }
  }

  out << attr_delim;
  out << _orientation.x() << value_delim << _orientation.y() << value_delim
      << _orientation.z() << value_delim << _orientation.w();
  out << attr_delim;
  out << _type << attr_delim;

  for (size_t i = 0; i < _shape.size(); i++) 
  {
    out << _shape[i];

    if (i < _shape.size() - 1) 
    {
      out << value_delim;
    }
  }
  out << "\n";
}

} // namespace mav_planning