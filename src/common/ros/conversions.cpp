#include <mav_planning/common/ros/conversions.h>

namespace mav_planning
{   
    namespace convert
    {   

        SE3State toSE3State(const geometry_msgs::Pose& msg)
        {
            SE3State state;

            state.position[0] = msg.position.x;
            state.position[1] = msg.position.y;
            state.position[2] = msg.position.z;
            state.rotation    = toEigenQuat(msg.orientation);

            return state;
        }

        geometry_msgs::Pose toPoseMsg(const Point& pos, const Quaternion& rot)
        {   
            geometry_msgs::Pose msg;

            msg.position.x = pos[0];
            msg.position.y = pos[1];
            msg.position.z = pos[2];
            msg.orientation.x = rot.x();
            msg.orientation.y = rot.y();
            msg.orientation.z = rot.z();
            msg.orientation.w = rot.w();

            return msg;
        }

        geometry_msgs::Pose toPoseMsg(const SE3State& state)
        {
            Point p = state.position;
            Quaternion q = state.rotation;

            return toPoseMsg(p, q);
        }
        

        Eigen::Vector3d toEulerRPY(const Quaternion& quat)
        {
            return quat.toRotationMatrix().eulerAngles(0,1,2);
        }

        Quaternion toEigenQuat(const geometry_msgs::Quaternion& quat)
        {
            Quaternion q;

            q.x() = quat.x;
            q.y() = quat.y;
            q.z() = quat.z;
            q.w() = quat.w;

            return q;
        }

        Quaternion toEigenQuat(const double& yaw)
        {
            return Quaternion(Eigen::AngleAxisd(yaw, Point::UnitZ()));
        }

        Quaternion toEigenQuat(const double& roll, const double& pitch, const double& yaw)
        {
            Quaternion quat =  Eigen::AngleAxisd(roll, Point::UnitX())*
                               Eigen::AngleAxisd(pitch, Point::UnitY())*
                               Eigen::AngleAxisd(yaw, Point::UnitZ());

            return quat;

        }

        bool toCollisionGeometry(XmlRpc::XmlRpcValue& param, CollisionGeometry& shape)
        {   
            if(!param.hasMember("type") || !param.hasMember("size") || !param.hasMember("center") || !param.hasMember("rpy"))
            {
                ROS_ERROR("[toCollisionGeometry] Incorrect parameter structure!");
                return false;
            }
            else
            {
                if(param["type"].getType()!=XmlRpc::XmlRpcValue::Type::TypeInt ||
                (param["size"].getType()!=XmlRpc::XmlRpcValue::Type::TypeArray || param["size"].size()!=3)  ||
                (param["center"].getType()!=XmlRpc::XmlRpcValue::Type::TypeArray || param["center"].size()!=3)  ||
                (param["rpy"].getType()!=XmlRpc::XmlRpcValue::Type::TypeArray || param["rpy"].size()!=3))
                {
                    ROS_ERROR("[toCollisionGeometry] Incorrect parameter structure!");
                    return false;
                }
                else
                {   
                    std::vector<double> size;
                    Point pos;
                    Quaternion quat;
                    
                    size.resize(param["size"].size());

                    for(unsigned i = 0; i<param["size"].size(); i++)
                    {
                        size[i] = (double) static_cast<double>(param["size"][i]);
                    }

                    for(unsigned i = 0; i< 3; i++)
                    {
                        pos[i] = static_cast<double>(param["center"][i]);
                    }

                    quat =  toEigenQuat((double) static_cast<double>(param["rpy"][0]),
                                        (double) static_cast<double>(param["rpy"][1]),
                                        (double) static_cast<double>(param["rpy"][2]));

                    auto type_enum = static_cast<CollisionGeometry::Type>(static_cast<int>(param["type"]));

                    switch(type_enum)
                    {   
                        case CollisionGeometry::Type::SPHERE:
                        case CollisionGeometry::Type::BOX:
                        case CollisionGeometry::Type::CYLINDER:
                        case CollisionGeometry::Type::ELLIPSOID:
                            shape = CollisionGeometry(type_enum, pos, quat, size);
                            break;

                        default:
                            ROS_ERROR("[toCollisionGeometry] Invalid geometry type!");
                            break;
                        
                    }

                    return true;
                }
            }
        }
    }   
}