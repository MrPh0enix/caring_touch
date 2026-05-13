#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/robot_state.h>
#include <franka/duration.h>

// Key listener
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <thread>
#include <array>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <Eigen/Dense>
#include "utils/pose_mover.h"
#include "utils/examples_common.h"





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
std::atomic<bool> running{false};



void keyListener() {

    termios newT, oldT;
    tcgetattr(STDIN_FILENO, &oldT); //current terminal settings for backup

    newT = oldT;
    newT.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newT);

    // Make thread non blocking
    int oldFlags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldFlags | O_NONBLOCK);

    char key;
    while(running.load()) {
        key = getchar();
        if (key == 'q' || key =='Q') {
            running.store(false);
            std::cout << "Q pressed ... " << std::endl;
        }
        usleep(10000); //delay
    }

    // restore terminal on exit
    tcsetattr(STDIN_FILENO, TCSANOW, &oldT);
    fcntl(STDIN_FILENO, F_SETFL, oldFlags);

}




auto INIT_run = [](franka::Robot& robot) {

    running.store(true);
    // key listener thread
    std::thread key_thread(keyListener);

    robot.control([](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
        
        if (!running.load()) {

            std::cout << "Exiting .... " << std::endl;
            
            return franka::MotionFinished(franka::Torques({0, 0, 0, 0, 0, 0, 0}));
            
        }

        std::array<double, 7> tau_cmd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        
        return tau_cmd;
        
    });

    key_thread.join();

};

auto CALIB_run = [](franka::Robot& robot) {

    franka::RobotState state = robot.readOnce();
    Eigen::Map<const Eigen::Matrix4d> T_p(state.O_T_EE.data());
    Eigen::Vector3d target_position = T_p.block<3,1>(0,3);

    std::array<double, 16> initial_pose = initial_state.O_T_EE;
    Eigen::Map<const Eigen::Matrix4d> T_r(initial_pose.data());
    Eigen::Matrix3d R = T_r.block<3,3>(0,0);

    Eigen::Matrix4d new_pose = Eigen::Matrix4d::Identity();
    new_pose.block<3,3>(0,0) = R;
    new_pose.block<3,1>(0,3) = target_position;
    std::array<double, 16> pose_array;
    Eigen::Map<Eigen::Matrix4d>(pose_array.data()) = new_pose;

    PoseMover mover(pose_array, 10.0);
    robot.control(mover);

};

auto APPROACH1_run = [](franka::Robot& robot) {

};


bool run_motion(franka::Robot& robot, State& state) {

    while (true) {

        switch(state) {

            case State::INIT:
                std::cout<<"-------------------------------------------------"<<std::endl;
                std::cout<<"Move the robot to the desired position"<<std::endl;
                std::cout<<"Press q when done ...."<<std::endl;
                INIT_run(robot);
                state = State::CALIB;

            case State::CALIB:
                std::cout<<"-------------------------------------------------"<<std::endl;
                std::cout<<"Initial calibration of robot"<<std::endl;
                CALIB_run(robot);
                std::cout<<"Calibration done...."<<std::endl;
                state = State::APPROACH1;

            // case State::APPROACH1:


        }

    }

};





int main() {
    
    try {
        franka::Robot robot("172.22.2.4"); // change IP

        std::cout << "Connected to robot" << std::endl; 

        // set collision behavior
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                                    {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

        
        // move robot to start
        const std::array<double, 7>  home_pos = {0.0, -0.78539816, 0.0, -2.35619449, 0.0, 1.57079633, 0.78539816};
        MotionGenerator motion_generator(0.5, home_pos);
        robot.control(motion_generator);


        franka::RobotState initial_state = robot.readOnce();


        State state = State::INIT;
        run_motion(robot, state);


    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
    }

    return 0;
}