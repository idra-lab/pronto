#include "pronto_quadruped/ForceSensorStanceEstimator.hpp"

using namespace pronto::quadruped;



bool ForceSensorStanceEstimator::getStance(LegBoolMap& stance,
                                           LegScalarMap& stance_probability) {

  stance[LF] = grf_[LF](2) > leg_thresholds_[LF];
  stance[RF] = grf_[RF](2) > leg_thresholds_[RF];
  stance[LH] = grf_[LH](2) > leg_thresholds_[LH];
  stance[RH] = grf_[RH](2) > leg_thresholds_[RH];

  stance_probability[LF] = static_cast<double>(stance[LF]);
  stance_probability[RF] = static_cast<double>(stance[RF]);
  stance_probability[LH] = static_cast<double>(stance[LH]);
  stance_probability[RH] = static_cast<double>(stance[RH]);

  return true;

}

bool ForceSensorStanceEstimator::getStance(LegBoolMap &stance) {
  LegScalarMap stance_probability;
  return getStance(stance, stance_probability);
}

bool ForceSensorStanceEstimator::getGRF(LegVectorMap& grf) {
  grf = grf_;
  return true;
}

void ForceSensorStanceEstimator::setGRF(const LegVectorMap& grf) {
    grf_[LF] = grf[LF] + Eigen::Vector3d(0,0,leg_sensor_offsets_[LF]);
    grf_[RF] = grf[RF] + Eigen::Vector3d(0,0,leg_sensor_offsets_[RF]);
    grf_[LH] = grf[LH] + Eigen::Vector3d(0,0,leg_sensor_offsets_[LH]);
    grf_[RH] = grf[RH] + Eigen::Vector3d(0,0,leg_sensor_offsets_[RH]);
}

bool ForceSensorStanceEstimator::isStance(LegID leg) const {
  return stance_[leg];
}
