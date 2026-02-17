#include "pronto_quadruped_ros/force_sensor_stance_estimator_ros.hpp"

namespace pronto {
namespace quadruped {

ForceSensorStanceEstimatorROS::ForceSensorStanceEstimatorROS(
    ros::NodeHandle &nh) {
  // get parameters for the leg odometry
  std::string legodo_prefix = "legodo/";
  double stance_threshold;

  nh.getParam(legodo_prefix + "lf_sensor_offset", leg_sensor_offsets_[LF]);
  nh.getParam(legodo_prefix + "rf_sensor_offset", leg_sensor_offsets_[RF]);
  nh.getParam(legodo_prefix + "lh_sensor_offset", leg_sensor_offsets_[LH]);
  nh.getParam(legodo_prefix + "rh_sensor_offset", leg_sensor_offsets_[RH]);

  int stance_mode;
  if (!nh.getParam(legodo_prefix + "stance_mode", stance_mode)) {
    ROS_WARN(
        "Could not read the stance mode from param server. Using threshold "
        "with default 50 N.");
    use_hysteresis_ = false;
  } else if (stance_mode == 1) {
    use_hysteresis_ = true;
  } else {
    ROS_WARN(
        "Invalid stance mode from param server. Using threshold with default "
        "50 N.");
    use_hysteresis_ = false;
  }

  int stance_hysteresis_delay_low_int = 0;
  int stance_hysteresis_delay_high_int = 0;

  if (!use_hysteresis_) {
    if (!nh.getParam(legodo_prefix + "stance_threshold", stance_threshold)) {
      ROS_WARN(
          "Could not read the stance threshold from param server. Using "
          "default 50 N.");
    }
  } else {
    if (!nh.getParam(legodo_prefix + "stance_hysteresis_low",
                      falling_edge_threshold_)) {
      ROS_WARN(
          "Could not read the stance_hysteresis_low from param server. Using "
          "default 50 N.");
    }
    if (!nh.getParam(legodo_prefix + "stance_hysteresis_high",
                      rising_edge_threshold_)) {
      ROS_WARN(
          "Could not read the stance_hysteresis_high from param server. Using "
          "default 50 N.");
    }
    if (!nh.getParam(legodo_prefix + "stance_hysteresis_delay_low",
                      stance_hysteresis_delay_low_int)) {
      ROS_WARN(
          "Could not read the stance_hysteresis_delay_low from param server. "
          "Using default 0 ns.");
    }
    if (!nh.getParam(legodo_prefix + "stance_hysteresis_delay_high",
                      stance_hysteresis_delay_high_int)) {
      ROS_WARN(
          "Could not read the stance_hysteresis_delay_high from param server. "
          "Using default 0 ns.");
    }

    falling_edge_delay_ = stance_hysteresis_delay_low_int;
    rising_edge_delay_ = stance_hysteresis_delay_high_int;
  }



  // force_threshold_ = stance_threshold;
  ROS_INFO_STREAM("[ForceSensorStanceEstimator] Stance threshold: "
                  << force_threshold_);
  ROS_INFO_STREAM(
      " ----------------------------------------------------------- ");
  ROS_INFO_STREAM("[ForceSensorStanceEstimator] LF sensor offset: "
                  << leg_sensor_offsets_[LF]);
  ROS_INFO_STREAM("[ForceSensorStanceEstimator] RF sensor offset: "
                  << leg_sensor_offsets_[RF]);
  ROS_INFO_STREAM("[ForceSensorStanceEstimator] LH sensor offset: "
                  << leg_sensor_offsets_[LH]);
  ROS_INFO_STREAM("[ForceSensorStanceEstimator] RH sensor offset: "
                  << leg_sensor_offsets_[RH]);
}

}  // namespace quadruped
}  // namespace pronto
