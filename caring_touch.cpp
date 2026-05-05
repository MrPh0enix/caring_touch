#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/robot_state.h>
#include <franka/duration.h>

#include <array>
#include <cmath>
#include <iostream>

enum class State {
    SIN_X,
    SIN_Y,
    SIN_Z
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

        State current_state = State::SIN_X;

        double state_start_time = 0.0;
        double total_time = 0.0;

        const double duration_per_state = 5.0;  // seconds
        const double amplitude = 0.05;          // 5 cm
        const double frequency = 0.5;           // Hz

        std::array<double, 16> initial_pose{};

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