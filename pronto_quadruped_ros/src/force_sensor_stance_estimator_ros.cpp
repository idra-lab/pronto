#include "pronto_quadruped_ros/force_sensor_stance_estimator_ros.hpp"


namespace pronto {
namespace quadruped {

ForceSensorStanceEstimatorROS::ForceSensorStanceEstimatorROS(double force_threshold)
  : ForceSensorStanceEstimator(force_threshold)
{
}

ForceSensorStanceEstimatorROS::ForceSensorStanceEstimatorROS(ros::NodeHandle &nh)
{
  // get parameters for the leg odometry
  std::string legodo_prefix = "legodo/";
  //double stance_threshold;
  nh.getParam(legodo_prefix + "lf_stance_threshold", leg_thresholds_[LF]);
  nh.getParam(legodo_prefix + "rf_stance_threshold", leg_thresholds_[RF]);
  nh.getParam(legodo_prefix + "lh_stance_threshold", leg_thresholds_[LH]);
  nh.getParam(legodo_prefix + "rh_stance_threshold", leg_thresholds_[RH]);

  nh.getParam(legodo_prefix + "lf_sensor_offset", leg_sensor_offsets_[LF]);
  nh.getParam(legodo_prefix + "rf_sensor_offset", leg_sensor_offsets_[RF]);
  nh.getParam(legodo_prefix + "lh_sensor_offset", leg_sensor_offsets_[LH]);
  nh.getParam(legodo_prefix + "rh_sensor_offset", leg_sensor_offsets_[RH]);

  //force_threshold_ = stance_threshold;
  ROS_INFO_STREAM("[ForceSensorStanceEstimator] LF stance threshold: " << leg_thresholds_[LF]);
  ROS_INFO_STREAM("[ForceSensorStanceEstimator] RF stance threshold: " << leg_thresholds_[RF]);
  ROS_INFO_STREAM("[ForceSensorStanceEstimator] LH stance threshold: " << leg_thresholds_[LH]);
  ROS_INFO_STREAM("[ForceSensorStanceEstimator] RH stance threshold: " << leg_thresholds_[RH]);
  ROS_INFO_STREAM(" ----------------------------------------------------------- ");
  ROS_INFO_STREAM("[ForceSensorStanceEstimator] LF sensor offset: " << leg_sensor_offsets_[LF]);
  ROS_INFO_STREAM("[ForceSensorStanceEstimator] RF sensor offset: " << leg_sensor_offsets_[RF]);
  ROS_INFO_STREAM("[ForceSensorStanceEstimator] LH sensor offset: " << leg_sensor_offsets_[LH]);
  ROS_INFO_STREAM("[ForceSensorStanceEstimator] RH sensor offset: " << leg_sensor_offsets_[RH]);
}

}  // namespace quadruped
}  // namespace pronto
