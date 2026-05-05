#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/robot_state.h>
#include <franka/duration.h>

#include <array>
#include <cmath>
#include <iostream>
#include <unordered_map>

enum class State {
    INIT,
    CALIB,
    APPROACH1,
    STEPFORCE1,
    STEPFORCE2,
    STEPFORCE3,
    RAMPFORCE1,
    RAMPFORCE2,
    RAMPFORCE3,
    RAMPFORCE4,
    SINFORCE1,
    SINFORCE2,
    SINFORCE3,
    SINFORCE4,
    SINFORCE5,
    SWEEP1,
    SWEEP2,
    SWEEP3
};


auto INIT_run = [&](franka::Robot& robot) {
    robot.control([&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
            

        

        
    });
};



int main() {
    
    try {
        franka::Robot robot("127.0.0.1"); // change IP

        std::cout << "Connected to robot" << std::endl; 

        // Set collision behavior (required)
        robot.setCollisionBehavior(
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
            {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}
        );

        State current_state = State::INIT;

        franka::RobotState initial_state = robot.readOnce();


        auto trq_ctrl_loop = [&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
            



        
        };


        auto cartesian_pose_ctrl_loop = [&](const franka::RobotState& state, franka::Duration period) -> franka::CartesianPose {
        
            

        
        };



        robot.control([&](const franka::RobotState& state,
                          franka::Duration period) -> franka::CartesianPose {

            double dt = period.toSec();
            total_time += dt;

            if (total_time == dt) {
                initial_pose = state.O_T_EE_c;
                state_start_time = total_time;
            }

            double t = total_time - state_start_time;

            // State transition
            if (t > duration_per_state) {
                state_start_time = total_time;

                switch (current_state) {
                    case State::SIN_X: current_state = State::SIN_Y; break;
                    case State::SIN_Y: current_state = State::SIN_Z; break;
                    case State::SIN_Z: current_state = State::SIN_X; break;
                }
            }

            // Copy pose
            std::array<double, 16> pose = initial_pose;

            double omega = 2 * M_PI * frequency;
            double offset = amplitude * std::sin(omega * t);

            // Apply sinusoid depending on state
            switch (current_state) {
                case State::SIN_X:
                    pose[12] = initial_pose[12] + offset;
                    break;

                case State::SIN_Y:
                    pose[13] = initial_pose[13] + offset;
                    break;

                case State::SIN_Z:
                    pose[14] = initial_pose[14] + offset;
                    break;
            }

            return franka::CartesianPose(pose);
        });

    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
    }

    return 0;
}