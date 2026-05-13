#pragma once

#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/duration.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <array>

class PoseMover {
public:
    PoseMover(const std::array<double, 16>& target_pose,
              double duration);

    void update(const franka::RobotState& state,
                franka::Duration period);

    Eigen::Vector3d getPosition() const;
    Eigen::Quaterniond getOrientation() const;

    bool isFinished() const;

private:
    std::array<double, 16> target_pose_;
    std::array<double, 16> start_pose_;

    Eigen::Quaterniond q_start_;
    Eigen::Quaterniond q_target_;

    Eigen::Vector3d position_d_;
    Eigen::Quaterniond q_d_;

    double duration_;
    double time_ = 0.0;
    bool finished_ = false;
};