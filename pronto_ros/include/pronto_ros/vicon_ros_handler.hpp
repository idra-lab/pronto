#pragma once
#include <ros/node_handle.h>
#include <pronto_core/vicon_module.hpp>
#include <geometry_msgs/TransformStamped.h>
#include <tf_conversions/tf_eigen.h>

namespace pronto {

class ViconHandlerROS : public SensingModule<geometry_msgs::PoseStamped> {
public:
    ViconHandlerROS(ros::NodeHandle& nh);

    RBISUpdateInterface* processMessage(const geometry_msgs::PoseStamped *msg,
                                        StateEstimator *est);

    bool processMessageInit(const geometry_msgs::PoseStamped *msg,
                            const std::map<std::string, bool> &sensor_initialized,
                            const RBIS &default_state,
                            const RBIM &default_cov,
                            RBIS &init_state,
                            RBIM &init_cov);


private:
    ros::NodeHandle& nh_;
    std::unique_ptr<MocapModule> vicon_module_;
    RigidTransform vicon_transf_;
};

} // namespace pronto
