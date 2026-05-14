#include <franka/robot.h>
#include <franka/exception.h>
#include <franka/robot_state.h>
#include <franka/duration.h>

// Key listener
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <atomic>

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



void moveToPose(franka::Robot& robot,
                const std::array<double, 16>& desired_pose,
                double duration = 10.0) {

    // franka::RobotState state0 = robot.readOnce();
    // static Eigen::Vector3d p_start;
    // static Eigen::Quaterniond q_start;

    // Eigen::Map<const Eigen::Matrix4d> T_start(state0.O_T_EE.data());
    Eigen::Map<const Eigen::Matrix4d> T_goal(desired_pose.data());

    // Eigen::Vector3d p_start = T_start.block<3,1>(0,3);
    Eigen::Vector3d p_goal  = T_goal.block<3,1>(0,3);
    
    // std::cout << T_start << std::endl;
    // std::cout << "------------------------------------------------" << std::endl;
    // std::cout << T_goal << std::endl;


    // Eigen::Quaterniond q_start(T_start.block<3,3>(0,0));
    Eigen::Quaterniond q_goal(T_goal.block<3,3>(0,0));


    robot.control([&](const franka::RobotState& state, franka::Duration period)
                  -> franka::CartesianPose {

        static bool init = false;
        static Eigen::Vector3d p_start;
        static Eigen::Quaterniond q_start;
        static double t = 0.0;

        t += period.toSec();
        
        std::cout << "------------------------------------------------" << std::endl;
        std::cout << "dt: " << period.toSec() << std::endl;

        if (!init) {
            Eigen::Map<const Eigen::Matrix4d> T(state.O_T_EE.data());
            p_start = T.block<3,1>(0,3);
            q_start = Eigen::Quaterniond(T.block<3,3>(0,0));
            init = true;
            t = 0.0;
        }

        double tau = std::min(t / duration, 1.0);

        
        std::cout << "------------------------------------------------" << std::endl;
        std::cout << "tau: " << tau << std::endl;

        // smoothstep (good for Franka)
        double s = tau * tau * (3 - 2 * tau);

        Eigen::Vector3d p = p_start + s * (p_goal - p_start);
        
        std::cout << "------------------------------------------------" << std::endl;
        std::cout << "pos error: " << (p_goal - p).norm() << std::endl;

        Eigen::Quaterniond q = q_start.slerp(s, q_goal);
        q.normalize();
        
        std::cout << "------------------------------------------------" << std::endl;
        double angle_error = q.angularDistance(q_goal);
        std::cout << "rot error (rad): " << angle_error << std::endl;

        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3,3>(0,0) = q.toRotationMatrix();
        T.block<3,1>(0,3) = p;

        std::cout << "------------------------------------------------" << std::endl;
        double vnorm = 0.0;
        for (const auto& v : state.dq) vnorm += v * v;
        vnorm = std::sqrt(vnorm);
        std::cout << "joint vel norm: " << vnorm << std::endl;

        std::array<double, 16> pose{};
        Eigen::Map<Eigen::Matrix4d>(pose.data()) = T;

        if (tau >= 1.0) {
            return franka::MotionFinished(franka::CartesianPose(pose));
        }

        return franka::CartesianPose(pose);
    });
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
    Eigen::Matrix3d R_initial = T_r.block<3,3>(0,0);

    Eigen::Matrix4d new_pose = Eigen::Matrix4d::Identity();
    new_pose.block<3,3>(0,0) = R_initial;
    new_pose.block<3,1>(0,3) = target_position;
    std::array<double, 16> pose_array;
    Eigen::Map<Eigen::Matrix4d>(pose_array.data()) = new_pose;

    moveToPose(robot, pose_array);

};



auto APPROACH1_run = [](franka::Robot& robot) {

    franka::RobotState state = robot.readOnce();
    Eigen::Map<const Eigen::Matrix4d> T(state.O_T_EE.data());
    Eigen::Vector3d target_position = T.block<3,1>(0,3);
    target_position(2) = target_position(2) + 0.16;

    Eigen::Matrix3d R = T.block<3,3>(0,0);

    Eigen::Matrix4d new_pose = Eigen::Matrix4d::Identity();
    new_pose.block<3,3>(0,0) = R;
    new_pose.block<3,1>(0,3) = target_position;
    std::array<double, 16> pose_array;
    Eigen::Map<Eigen::Matrix4d>(pose_array.data()) = new_pose;

    moveToPose(robot, pose_array, 5.0);


};


bool run_motion(franka::Robot& robot, State& state) {

    while (true) {

        if (state == State::INIT) {
            std::cout<<"-------------------------------------------------"<<std::endl;
            std::cout<<"Move the robot to the desired position"<<std::endl;
            std::cout<<"Press q when done ...."<<std::endl;
            INIT_run(robot);
            state = State::CALIB;
        }

        if (state == State::CALIB) {
            std::cout<<"-------------------------------------------------"<<std::endl;
            std::cout<<"Initial calibration of robot"<<std::endl;
            CALIB_run(robot);
            std::cout<<"Calibration done...."<<std::endl;
            state = State::APPROACH1;
        }

        if (state == State::APPROACH1) {
            std::cout<<"-------------------------------------------------"<<std::endl;
            std::cout<<"Robot approaching sample till threshold force"<<std::endl;
            APPROACH1_run(robot);
            std::cout<<"Contact Achieved...."<<std::endl;
            state = State::STEPFORCE1;
        }

    }

    return true;
};





int main() {
    
    try {
        franka::Robot robot("192.168.33.3"); // change IP

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


        initial_state = robot.readOnce();


        State state = State::CALIB;
        run_motion(robot, state);


    } catch (const franka::Exception& e) {
        std::cout << e.what() << std::endl;
    }

    return 0;
}