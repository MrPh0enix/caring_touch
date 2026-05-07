#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/robot_state.h>
#include <franka/duration.h>

#include <array>

class PoseMover {

public:
    PoseMover(const std::array<double, 16>& target_pose, double duration) 
    : target_pose_(target_pose), duration_(duration), time_(0.0), finished_(false) {}
    
    franka::CartesianPose operator()(const franka::RobotState& state, franka::Duration period) {

        if (time_ == 0.0) {
            start_pose_ = state.O_T_EE;
        }

        time_ += period.toSec();
        double alpha = std::min(time_ / duration_, 1.0);

        std::array<double,16> pose = start_pose_;
        
        for (int i = 12; i <= 14; i++) {
            pose[i] =
                start_pose_[i] +
                alpha * (target_pose_[i] - start_pose_[i]);
        }

        if (alpha >= 1.0 && !finished_) {
            finished_ = true;
            return franka::MotionFinished(
                franka::CartesianPose(pose));
        }
        
        return franka::CartesianPose(pose);

    };

private:
    std::array<double, 16> target_pose_;
    std::array<double, 16> start_pose_;
    double duration_;
    double time_;
    bool finished_;

};