#include "pronto_quadruped/ForceSensorStanceEstimator.hpp"

using namespace pronto::quadruped;
using iit::rbd::X;
using iit::rbd::Y;
using iit::rbd::Z;

bool ForceSensorStanceEstimator::getStance(LegBoolMap& stance,
                                           LegScalarMap& stance_probability) {
  if (!getGRF(grf_)) {
    return false;
  }
  // get the Ground Reaction Forces at the feet, expressed in the base frame
  for (int leg_id = 0; leg_id < _LEGS_COUNT; leg_id++) {
    if (!use_hysteresis_)
      stance[leg_id] = grf_[leg_id](Z) > force_threshold_ ? true : false;

    else {
      force_triggers_[leg_id].updateState(nsec_, grf_[leg_id](Z));
      stance[leg_id] = force_triggers_[leg_id].getState();
    }
  }

  stance_probability[LF] = static_cast<double>(stance[LF]);
  stance_probability[RF] = static_cast<double>(stance[RF]);
  stance_probability[LH] = static_cast<double>(stance[LH]);
  stance_probability[RH] = static_cast<double>(stance[RH]);

  return true;
}

bool ForceSensorStanceEstimator::getStance(LegBoolMap& stance) {
  LegScalarMap stance_probability;
  return getStance(stance, stance_probability);
}

bool ForceSensorStanceEstimator::getGRF(LegVectorMap& grf) {
  grf = grf_;
  return true;
}

void ForceSensorStanceEstimator::setGRF(const LegVectorMap& grf) {
  grf_[LF] = grf[LF] + Eigen::Vector3d(0, 0, leg_sensor_offsets_[LF]);
  grf_[RF] = grf[RF] + Eigen::Vector3d(0, 0, leg_sensor_offsets_[RF]);
  grf_[LH] = grf[LH] + Eigen::Vector3d(0, 0, leg_sensor_offsets_[LH]);
  grf_[RH] = grf[RH] + Eigen::Vector3d(0, 0, leg_sensor_offsets_[RH]);
}

bool ForceSensorStanceEstimator::isStance(LegID leg) const {
  return stance_[leg];
}
