#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/robot_state.h>
#include <franka/duration.h>

#include <array>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <Eigen/Dense>
#include "utils/pose_mover.h"


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


franka::RobotState initial_state;






auto INIT_run = [&](franka::Robot& robot) {

    robot.control([](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
            
        std::array<double, 7> tau_cmd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        
        return tau_cmd;
        
    });

};

auto CALIB_run = [&](franka::Robot& robot) {

    franka::RobotState state = robot.readOnce();
    Eigen::Map<const Eigen::Matrix4d> T(state.O_T_EE.data());
    Eigen::Vector3d target_position = T.block<3,1>(0,3);

    std::array<double, 16> initial_pose = initial_state.O_T_EE;

    // quat = R.from_matrix(init_pose).as_quat()
    double time = 0.0;

    PoseMover mover(target, 5.0);

    robot.control(mover);
    

};


bool run_motion(franka::Robot& robot, State& state) {

    while (true) {

        switch(state) {

            case State::INIT:
                std::cout<<"Move the robot to the desired position"<<std::endl;
                INIT_run(robot);

            case State::CALIB:
                std::cout<<"-------------------------------------------------"<<std::endl;
                std::cout<<"Initial calibration of robot"<<std::endl;
                std::cout<<"Press q when done ...."<<std::endl;
                CALIB_run(robot);

            case State::APPROACH1:


        }

    }

};





int main() {
    
    try {
        franka::Robot robot("127.0.0.1"); // change IP

        std::cout << "Connected to robot" << std::endl; 

        // set collision behavior
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});


        franka::RobotState initial_state = robot.readOnce();


        State state = State::INIT;
        run_motion(robot, state);


    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
    }

    return 0;
}