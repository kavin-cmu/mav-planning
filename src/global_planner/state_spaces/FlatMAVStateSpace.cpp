#include <mav_planning/global_planner/state_spaces/FlatMAVStateSpace.h>


namespace mav_planning{

void FlatMAVStateSpace::registerProjections(void) {
  class MAVDefaultProjection : public ob::ProjectionEvaluator
  {
   public:

    MAVDefaultProjection(const ob::StateSpace *space) : ob::ProjectionEvaluator(space)
    {
    }

    virtual unsigned int getDimension(void) const
    {
      return 3;
    }

    virtual void defaultCellSizes(void)
    {
      cellSizes_.resize(3);
      bounds_ = space_->as<FlatMAVStateSpace>()->getBounds();
      cellSizes_[0] = (bounds_.high[0] - bounds_.low[0]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
      cellSizes_[1] = (bounds_.high[1] - bounds_.low[1]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
      cellSizes_[2] = (bounds_.high[2] - bounds_.low[2]) / ompl::magic::PROJECTION_DIMENSION_SPLITS;
    }

    virtual void project(const ob::State *state, Eigen::Ref<Eigen::VectorXd> projection) const
    {
        projection = Eigen::Map<const Eigen::VectorXd>(state->as<FlatMAVStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values, 3);
        memcpy(&projection(0), state->as<FlatMAVStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(0)->values, 3 * sizeof(double));
    }
  };

  registerDefaultProjection(std::make_shared<MAVDefaultProjection>(this));
}
}
