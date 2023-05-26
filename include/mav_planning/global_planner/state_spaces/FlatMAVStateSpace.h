#ifndef GLOBAL_PLANNER_STATE_SPACES_FLATMAVSTATESPACE
#define GLOBAL_PLANNER_STATE_SPACES_FLATMAVSTATESPACE

#include <mav_planning/common/common.h>

#include <ompl/base/StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/tools/config/MagicConstants.h>

namespace ob = ompl::base;
// namespace og = ompl::geometric;

namespace mav_planning{

class FlatMAVStateSpace: public ob::CompoundStateSpace
{
    public:
        
        class StateType : public ob::CompoundStateSpace::StateType
        {
        
        public:
            StateType(void) : ob::CompoundStateSpace::StateType()
            {}

            /** \brief Get the X component of the state */
            float getX(void) const
            {
                return as<ob::RealVectorStateSpace::StateType>(0)->values[0];
            }

            /** \brief Get the Y component of the state */
            float getY(void) const
            {
                return as<ob::RealVectorStateSpace::StateType>(0)->values[1];
            }

            /** \brief Get the Z component of the state */
            float getZ(void) const
            {
                return as<ob::RealVectorStateSpace::StateType>(0)->values[2];
            }

            Point getXYZ() const
            {
                return Point(getX(), getY(), getZ());
            }

            /** \brief Get the yaw component of the state. This is
                the rotation in plane, with respect to the Z
                axis. */
            float getYaw(void) const
            {
                return as<ob::SO2StateSpace::StateType>(1)->value;
            }

            Quaternion getYawAsQuat() const
            {
                return Quaternion(Eigen::AngleAxisd(getYaw(), Point::UnitZ())); 
            }

            FlatMAVState getXYZYaw() const
            {
                return FlatMAVState(getX(), getY(), getZ(), getYaw());
            }

            const ob::RealVectorStateSpace::StateType& GetTranslation() const
            {
            return *as<ob::RealVectorStateSpace::StateType>(0);
            }

            const ob::SO2StateSpace::StateType& GetRotation() const
            {
            return *as<ob::SO2StateSpace::StateType>(1);
            }

            /** \brief Set the X component of the state */
            void setX(float x)
            {
                as<ob::RealVectorStateSpace::StateType>(0)->values[0] = x;
            }

            /** \brief Set the Y component of the state */
            void setY(float y) 
            {
                as<ob::RealVectorStateSpace::StateType>(0)->values[1] = y;
            }

            /** \brief Set the Y component of the state */
            void setZ(float z)
            {
                as<ob::RealVectorStateSpace::StateType>(0)->values[2] = z;
            }

            void setXYZ(const Point &pos) 
            {
                setX(pos.x());
                setY(pos.y());
                setZ(pos.z());
            }

            /** \brief Set the yaw component of the state. This is
                the rotation in plane, with respect to the Z
                axis. */
            void setYaw(float yaw)
            {
                as<ob::SO2StateSpace::StateType>(1)->value = yaw;
                float v = fmod(as<ob::SO2StateSpace::StateType>(1)->value, 2.0 * M_PI);
                if (v <= -M_PI)
                {
                    v += 2.0 * M_PI;
                }
                else
                {
                    if (v > M_PI)
                    {
                        v -= 2.0 * M_PI;
                    }    
                }
                as<ob::SO2StateSpace::StateType>(1)->value = v;
            }

            void setXYZYaw(FlatMAVState state)
            {
                setX(state[0]);
                setY(state[1]);
                setZ(state[2]);
                setYaw(state[3]);
            }

            ob::RealVectorStateSpace::StateType& GetTranslation()
            {
                return *as<ob::RealVectorStateSpace::StateType>(0);
            }

            ob::SO2StateSpace::StateType& GetRotation()
            {
                return *as<ob::SO2StateSpace::StateType>(1);
            }

        };


    FlatMAVStateSpace():
        CompoundStateSpace()
        {
            setName("FlatMAVStateSpace" + getName());
            type_ = ob::STATE_SPACE_SE3;

            ob::StateSpacePtr space;

            space = std::make_shared<ob::RealVectorStateSpace>(3);
            addSubspace(space, 1.0);

            space = std::make_shared<ob::SO2StateSpace>();
            addSubspace(space, 1.0);

            lock();
        }

    void setBounds(const ob::RealVectorBounds &bounds)
    {
        as<ob::RealVectorStateSpace>(0)->setBounds(bounds);
    }

    const ob::RealVectorBounds& getBounds(void) const   
    {
        return as<ob::RealVectorStateSpace>(0)->getBounds();
    }

    virtual ~FlatMAVStateSpace(void){};


    virtual void registerProjections(void);

};

}

#endif /* GLOBAL_PLANNER_STATE_SPACES_FLATMAVSTATESPACE */
