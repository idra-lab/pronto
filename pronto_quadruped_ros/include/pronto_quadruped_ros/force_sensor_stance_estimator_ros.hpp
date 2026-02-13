#pragma once

#include <pronto_quadruped/ForceSensorStanceEstimator.hpp>
#include <ros/node_handle.h>

namespace pronto {
namespace quadruped {

class ForceSensorStanceEstimatorROS : public ForceSensorStanceEstimator {
public:  
  ForceSensorStanceEstimatorROS(ros::NodeHandle& nh);
  ~ForceSensorStanceEstimatorROS() override {}
};

}  // namespace quadruped
}  // namespace pronto
