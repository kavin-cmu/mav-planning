#include "mav_planning/global_planner/state_spaces/state_utils.h"

namespace mav_planning::convert
{
    void toOMPLState(ob::ScopedStatePtr& state, const SE3State& eig_state, const StateSpaceType& type)
    {
        switch (type)
        {
            case StateSpaceType::XYZ_STATE_SPACE:
            {
                state->get()->as<ob::RealVectorStateSpace::StateType>()->values[0] = eig_state.position[0];
                state->get()->as<ob::RealVectorStateSpace::StateType>()->values[1] = eig_state.position[1];               
                state->get()->as<ob::RealVectorStateSpace::StateType>()->values[2] = eig_state.position[2];               

                break;
            }

            case StateSpaceType::FLAT_MAV_STATE_SPACE:
            {   
                state->get()->as<FlatMAVStateSpace::StateType>()->setXYZ(eig_state.position);
                state->get()->as<FlatMAVStateSpace::StateType>()->setYaw(convert::toEulerRPY(eig_state.rotation)[2]); 
                break;
            }

            case StateSpaceType::SE3_STATE_SPACE:
            {   
                state->get()->as<ob::SE3StateSpace::StateType>()->setX(eig_state.position[0]);
                state->get()->as<ob::SE3StateSpace::StateType>()->setY(eig_state.position[1]);
                state->get()->as<ob::SE3StateSpace::StateType>()->setZ(eig_state.position[2]);

                state->get()->as<ob::SE3StateSpace::StateType>()->rotation().x = eig_state.rotation.x();
                state->get()->as<ob::SE3StateSpace::StateType>()->rotation().y = eig_state.rotation.y();
                state->get()->as<ob::SE3StateSpace::StateType>()->rotation().z = eig_state.rotation.z();
                state->get()->as<ob::SE3StateSpace::StateType>()->rotation().w = eig_state.rotation.w(); 
                
                break;
            }

            default:
                throw ompl::Exception("Invalid state space type provided!");
                break;
        }
    }

    void fromOMPLState(const ob::ScopedStatePtr& state, SE3State& eig_state, const StateSpaceType& type)
    {
        switch (type)
        {
            case StateSpaceType::XYZ_STATE_SPACE:
            {
                eig_state.position[0] = state->get()->as<ob::RealVectorStateSpace::StateType>()->values[0];
                eig_state.position[1] = state->get()->as<ob::RealVectorStateSpace::StateType>()->values[1];               
                eig_state.position[2] = state->get()->as<ob::RealVectorStateSpace::StateType>()->values[2];
                eig_state.rotation.setIdentity();               

                break;
            }

            case StateSpaceType::FLAT_MAV_STATE_SPACE:
            {   
                eig_state.position = state->get()->as<FlatMAVStateSpace::StateType>()->getXYZ();
                eig_state.rotation = state->get()->as<FlatMAVStateSpace::StateType>()->getYawAsQuat();

                break;
            }

            case StateSpaceType::SE3_STATE_SPACE:
            {   
                eig_state.position[0] = state->get()->as<ob::SE3StateSpace::StateType>()->getX();
                eig_state.position[1] = state->get()->as<ob::SE3StateSpace::StateType>()->getY();
                eig_state.position[2] = state->get()->as<ob::SE3StateSpace::StateType>()->getZ();

                eig_state.rotation.x() = state->get()->as<ob::SE3StateSpace::StateType>()->rotation().x;
                eig_state.rotation.y() = state->get()->as<ob::SE3StateSpace::StateType>()->rotation().y;
                eig_state.rotation.z() = state->get()->as<ob::SE3StateSpace::StateType>()->rotation().z;
                eig_state.rotation.w() = state->get()->as<ob::SE3StateSpace::StateType>()->rotation().w;
                break;
            }

            default:
                throw ompl::Exception("Invalid state space type provided!");
                break;
        }
    }
}