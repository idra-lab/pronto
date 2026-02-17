#pragma once
#include "pronto_quadruped/StanceEstimatorBase.hpp"
#include <pronto_utils/SchmittTrigger.hpp>

namespace pronto {

namespace quadruped {

class ForceSensorStanceEstimator : public StanceEstimatorBase {
public:
  ForceSensorStanceEstimator(double /*force_threshold*/ = 50) {
  }

  bool getStance(LegBoolMap &stance) override;

  bool getStance(LegBoolMap& stance,
                 LegScalarMap& stance_probability) override;

  bool getGRF(LegVectorMap& grf) override;

  void setGRF(const LegVectorMap& grf) override;

  bool isStance(LegID leg) const override;

  void setJointStates(const uint64_t& nsec,
                              const JointState& /*q*/,
                              const JointState& /*qd*/,
                              const JointState& /*tau*/,
                              const Quaterniond& /*orient*/,
                              const JointState& /*qdd*/ = JointState::Constant(0),
                              const Vector3d& /*xd*/ = Vector3d(0, 0, 0),
                              const Vector3d& /*xdd*/  = Vector3d(0, 0, 0),
                              const Vector3d& /*omega*/  = Vector3d(0, 0, 0),
                      const Vector3d& /*omegad*/ = Vector3d(0, 0, 0)) {
      nsec_ = nsec;
  }

protected:
  LegBoolMap stance_;
  LegVectorMap grf_;
  LegDataMap<int> leg_sensor_offsets_;
  double force_threshold_;
  bool use_hysteresis_ = false;
  double falling_edge_threshold_;
  double rising_edge_threshold_;
  double falling_edge_delay_;
  double rising_edge_delay_;
  uint64_t nsec_; // time in nanoseconds

  LegDataMap<SchmittTrigger> force_triggers_;

};

}  // namespace quadruped
}  // namespace pronto
