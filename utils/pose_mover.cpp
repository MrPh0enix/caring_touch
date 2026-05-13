#include "pose_mover.h"

#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>

PoseMover::PoseMover(
    const std::array<double, 16>& target_pose,
    double duration)
    : target_pose_(target_pose),
      duration_(duration),
      time_(0.0),
      finished_(false) {}

void PoseMover::update(
    const franka::RobotState& state,
    franka::Duration period) {

    if (time_ == 0.0) {
        start_pose_ = state.O_T_EE;

        Eigen::Map<const Eigen::Matrix4d> T_start(start_pose_.data());
        Eigen::Map<const Eigen::Matrix4d> T_target(target_pose_.data());

        Eigen::Matrix3d R_start = T_start.block<3,3>(0,0);
        Eigen::Matrix3d R_target = T_target.block<3,3>(0,0);

        q_start_  = Eigen::Quaterniond(R_start);
        q_target_ = Eigen::Quaterniond(R_target);

        if (q_start_.coeffs().dot(q_target_.coeffs()) < 0.0) {
            q_target_.coeffs() *= -1.0;
        }
    }

    time_ += period.toSec();

    double alpha = std::min(time_ / duration_, 1.0);

    // smoothstep
    alpha = alpha * alpha * (3.0 - 2.0 * alpha);

    Eigen::Map<const Eigen::Matrix4d> T_start(start_pose_.data());
    Eigen::Map<const Eigen::Matrix4d> T_target(target_pose_.data());

    Eigen::Vector3d p_start = T_start.block<3,1>(0,3);
    Eigen::Vector3d p_target = T_target.block<3,1>(0,3);

    position_d_ = p_start + alpha * (p_target - p_start);

    q_d_ = q_start_.slerp(alpha, q_target_);

    if (alpha >= 1.0) {
        finished_ = true;
    }
};