

import panda_py
import panda_py.controllers
import numpy as np
import yaml
import keyboard
from ikpy.chain import Chain
from scipy.spatial.transform import Rotation as R
import time
import math
import threading
import csv
import queue

with open("config.yml", "r") as file:
    config = yaml.safe_load(file)


robot = panda_py.Panda(config["robot"]["ip"])
robot_settings = robot.get_robot()
robot_settings.set_collision_behavior(lower_torque_thresholds_acceleration = [x / 10 for x in [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                    upper_torque_thresholds_acceleration = [x * 10 for x in[20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                    lower_torque_thresholds_nominal = [x / 10 for x in [20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                    upper_torque_thresholds_nominal = [x * 10 for x in[20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0]],
                                    lower_force_thresholds_acceleration = [x / 10 for x in [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]], 
                                    upper_force_thresholds_acceleration = [x * 10 for x in[20.0, 20.0, 20.0, 25.0, 25.0, 25.0]],
                                    lower_force_thresholds_nominal = [x / 10 for x in [20.0, 20.0, 20.0, 25.0, 25.0, 25.0]],
                                    upper_force_thresholds_nominal = [x * 10 for x in[20.0, 20.0, 20.0, 25.0, 25.0, 25.0]])


robot.move_to_start()
init_state = robot.get_state()
init_pose = np.array(init_state.O_T_EE).reshape(4, 4).T[:3, :3]
contact_pos = None
z_force_offset = None




states = {
    'INIT': 'Move the robot to the desired position',
    'CALIB': 'Initial calibration of robot',
    'APPROACH1': 'Robot approaching sample till threshold force',
    'STEPFORCE1': 'A step force reference of 2 N actuating at 𝑡 = 1 s and lasting for 9 s ',
    'STEPFORCE2': 'A step force reference of 5 N actuating at 𝑡 = 1 s and lasting for 9 s ',
    'STEPFORCE3': 'A step force reference of 8 N actuating at 𝑡 = 1 s and lasting for 9 s ', 
    'RAMPFORCE1': 'A ramp force reference from 2 N to 8 N in 6 seconds acting at 𝑡 = 2 s',
    'RAMPFORCE2': 'A ramp force reference from 2 N to 8 N in 3 seconds acting at 𝑡 = 2 s',
    'RAMPFORCE3': 'A ramp force reference from 2 N to 8 N in 1.2 seconds acting at 𝑡 = 2 s',
    'RAMPFORCE4': 'A ramp force reference from 2 N to 8 N in 0.6 seconds acting at 𝑡 = 2 s',
    'SINFORCE1': 'A sinus force ref of 𝐹 = 5 + 3sin(𝑡 − 1) N for 8 seconds acting at 𝑡 = 1 s',
    'SINFORCE2': 'A sinus force ref of 𝐹 = 5 + 3sin(2𝑡 − 2) N for 8 seconds acting at 𝑡 = 1 s',
    'SINFORCE3': 'A sinus force ref of 𝐹 = 5 + 3sin(3𝑡 − 3) N for 8 seconds acting at 𝑡 = 1 s',
    'SINFORCE4': 'A sinus force ref of 𝐹 = 5 + 3sin(4𝑡 − 4) N for 8 seconds acting at 𝑡 = 1 s',
    'SINFORCE5': 'A sinus force ref of 𝐹 = 5 + 3sin(5𝑡 − 5) N for 8 seconds acting at 𝑡 = 1 s',
    'SWEEP1': 'A sweep ref of 𝐹 = 5 + 3sin((𝑡 − 1)ଶ) N for 8 seconds acting at 𝑡 = 1 s',
    'SWEEP2': 'A sweep ref of 𝐹 = 5 + 3sin((2𝑡 − 2)ଶ) N for 8 seconds acting at 𝑡 = 1 s',
    'SWEEP3': 'A sweep ref of 𝐹 = 5 + 3sin((3𝑡 − 3)ଶ) N for 8 seconds acting at 𝑡 = 1 s',
}



trqController = panda_py.controllers.AppliedTorque()
posController = panda_py.controllers.JointPosition()
impController = panda_py.controllers.CartesianImpedance(
                                                        impedance=np.array([
                                                            [600, 0, 0, 0, 0, 0],
                                                            [0, 600, 0, 0, 0, 0],
                                                            [0, 0, 600, 0, 0, 0],
                                                            [0, 0, 0, 80, 0, 0],
                                                            [0, 0, 0, 0, 80, 0],
                                                            [0, 0, 0, 0, 0, 80],
                                                            ]),
                                                        damping_ratio=1.0,
                                                        nullspace_stiffness=0.5
                                                    )
frcController = panda_py.controllers.AppliedForce()









def run(state):

    if state == 'INIT':
        print('-------------------------------------------------')
        print(states[state])
        print('Press q when done....')
        robot.start_controller(trqController)
        INIT_run(trqController)

    elif state == 'CALIB':
        print('-------------------------------------------------')
        print(states[state], ' DO NOT TOUCH WHEN MOVING')
        print(' When it has stopped moving. See if its vertical and adjust position. ')
        print('Press q when done ....')
        
        CALIB_run()
    
    elif state == 'APPROACH1':
        print('-------------------------------------------------')
        print(states[state], 'DO NOT TOUCH WHEN MOVING')

        APPROACH1_run()

        print('Robot has reached contact')
    
    elif state == 'STEPFORCE1':

        print('-------------------------------------------------')
        print('Starting profile:')
        print(states[state])

        filename = 'recording/A1.csv'

        STEPFORCE_run(2, filename)

    elif state == 'STEPFORCE2':

        print('-------------------------------------------------')
        print('Starting profile:')
        print(states[state])

        filename = 'recording/A2.csv'

        STEPFORCE_run(5, filename)
    
    elif state == 'STEPFORCE3':

        print('-------------------------------------------------')
        print('Starting profile:')
        print(states[state])

        filename = 'recording/A3.csv'

        STEPFORCE_run(8, filename)

    elif state == 'RAMPFORCE1':

        print('-------------------------------------------------')
        print('Starting profile:')
        print(states[state])

        filename = 'recording/B1.csv'
        
        RAMPFORCE_run(6, filename)
    
    elif state == 'RAMPFORCE2':

        print('-------------------------------------------------')
        print('Starting profile:')
        print(states[state])

        filename = 'recording/B2.csv'
        
        RAMPFORCE_run(3, filename)

    elif state == 'RAMPFORCE3':

        print('-------------------------------------------------')
        print('Starting profile:')
        print(states[state])

        filename = 'recording/B3.csv'
        
        RAMPFORCE_run(1.2, filename)
    
    elif state == 'RAMPFORCE4':

        print('-------------------------------------------------')
        print('Starting profile:')
        print(states[state])

        filename = 'recording/B4.csv'
        
        RAMPFORCE_run(0.6, filename)
    
    elif state == 'SINFORCE1':

        print('-------------------------------------------------')
        print('Starting profile:')
        print(states[state])

        filename = 'recording/C1.csv'

        SINFORCE_run(1, filename)
    
    elif state == 'SINFORCE2':

        print('-------------------------------------------------')
        print('Starting profile:')
        print(states[state])

        filename = 'recording/C2.csv'

        SINFORCE_run(2, filename)

    elif state == 'SINFORCE3':

        print('-------------------------------------------------')
        print('Starting profile:')
        print(states[state])

        filename = 'recording/C3.csv'

        SINFORCE_run(3, filename)

    elif state == 'SINFORCE4':

        print('-------------------------------------------------')
        print('Starting profile:')
        print(states[state])

        filename = 'recording/C4.csv'

        SINFORCE_run(4, filename)

    elif state == 'SINFORCE5':

        print('-------------------------------------------------')
        print('Starting profile:')
        print(states[state])

        filename = 'recording/C5.csv'

        SINFORCE_run(5, filename)

    elif state == 'SWEEP1':

        print('-------------------------------------------------')
        print('Starting profile:')
        print(states[state])

        filename = 'recording/D1.csv'

        SWEEP_run(1, filename)

    elif state == 'SWEEP2':

        print('-------------------------------------------------')
        print('Starting profile:')
        print(states[state])

        filename = 'recording/D2.csv'

        SWEEP_run(2, filename)
    
    elif state == 'SWEEP3':

        print('-------------------------------------------------')
        print('Starting profile:')
        print(states[state])

        filename = 'recording/D3.csv'

        SWEEP_run(3, filename)
    
    
    



data_queue = queue.Queue()
def recording_thread(filename):

    with open(filename, mode="w", newline="") as file:
        writer = csv.writer(file)

        writer.writerow(["time", "x", "y", "z", "fz", 'q', 'r'])

        while not data_queue.empty():

            try:
                row = data_queue.get(timeout=0.1)
                writer.writerow(row)

            except queue.Empty:
                continue
def record(robot_state):
    global data_queue
    data_queue.put(robot_state.q)




def INIT_run(controller):

    with robot.create_context(frequency = config['robot']['operating_freq']) as ctx:

        while ctx.ok():

            if keyboard.is_pressed('q'):
                robot.stop_controller()
                print('stopping...')
                break

            trq = np.array([0] * 7)
            controller.set_control(trq)


def CALIB_run():

    state = robot.get_state()

    target_position = state.O_T_EE[12:15]
    
    quat = R.from_matrix(init_pose).as_quat()

    robot.move_to_pose(
        target_position,
        quat,
        speed_factor=0.2,
        success_threshold = 0.001,
        dq_threshold = 0.001,

        impedance = np.array([
        [500, 0,   0,   0,  0,  0],
        [0,   500, 0,   0,  0,  0],
        [0,   0,   500, 0,  0,  0],
        [0,   0,   0,   180, 0,  0],
        [0,   0,   0,   0,  180, 0],
        [0,   0,   0,   0,  0,  80],
        ])
    )

    state = robot.get_state()

    robot.start_controller(impController)
    

    with robot.create_context(frequency = config['robot']['operating_freq']) as ctx:

        while ctx.ok():

            if keyboard.is_pressed('q'):
                robot.stop_controller()
                break

            impController.set_control(position=target_position, orientation=quat, q_nullspace=state.q)


def APPROACH1_run():

    global contact_pos, z_force_offset

    state = robot.get_state()
    quat = R.from_matrix(init_pose).as_quat()
    target_position = state.O_T_EE[12:15]
    target_position = np.array([target_position[0], target_position[1], config['load_cell']['height']])
    robot.move_to_pose(
        target_position,
        quat,
        speed_factor=0.1,
        success_threshold = 0.001,
        dq_threshold = 0.001,

        impedance = np.array([
        [500, 0,   0,   0,  0,  0],
        [0,   500, 0,   0,  0,  0],
        [0,   0,   500, 0,  0,  0],
        [0,   0,   0,   180, 0,  0],
        [0,   0,   0,   0,  180, 0],
        [0,   0,   0,   0,  0,  80],
        ])
    )

    time.sleep(6)

    z_force_offset = state.K_F_ext_hat_K[2]


    robot.start_controller(impController)

    state = robot.get_state()

    T = np.array(state.O_T_EE).reshape(4, 4).T
    position = T[:3, 3]
    orientation = T[:3, :3]

    quat = R.from_matrix(orientation).as_quat()

    with robot.create_context(frequency = config['robot']['operating_freq']) as ctx:

        dt = 1.0 / config['robot']['operating_freq']
        dz = -0.002 * dt  # convert m/s → per-step displacement

        threshold = 1.0 - z_force_offset
        

        moving = True
        Fz_filt = 0.0
        alpha = 0.1

        prev_Fz = z_force_offset

        print('Moving...')

        while ctx.ok():

            state = robot.get_state()

            Fz = state.K_F_ext_hat_K[2]
            Fz_filt = alpha * Fz + (1 - alpha) * Fz_filt
            dFz = Fz_filt - prev_Fz
            prev_Fz = Fz_filt

            if moving and (Fz_filt > threshold and abs(dFz) < 0.005):
                print("Contact detected. Stopping motion.")
                robot.stop_controller()
                moving = False
                dz = 0.0
                break

            
            position = position + np.array([0.0, 0.0, dz])

            
            impController.set_control(
                position=position,
                orientation=quat,
                q_nullspace=state.q
            )
    
        state = robot.get_state()
        contact_pos = np.array(state.q)

    
def STEPFORCE_run(force, filename):
    
    robot.move_to_joint_position(contact_pos)

    time.sleep(2)

    robot.start_controller(frcController)

    with robot.create_context(frequency = config['robot']['operating_freq']) as ctx:

        start_time = time.time()

        rec_thread = threading.Thread(target=recording_thread, args=(filename,))
        rec_thread.start()

        while ctx.ok():

            state = robot.get_state()
            record(state)
            t = time.time() - start_time

            if t < 1.0:
                force_command = np.zeros(6)

            elif t < 10.0:
                force_command = np.array([0, 0, -force, 0, 0, 0])
            
            else:
                robot.stop_controller()
                break
            
            frcController.set_control(force_command)
        
        while True:
            if data_queue.empty():
                rec_thread.join()
                break
    
    robot.move_to_joint_position(contact_pos)
        

def RAMPFORCE_run(act_time, filename):

    robot.move_to_joint_position(contact_pos)

    time.sleep(2)

    robot.start_controller(frcController)


    with robot.create_context(frequency = config['robot']['operating_freq']) as ctx:

        start_time = time.time()

        rec_thread = threading.Thread(target=recording_thread, args=(filename,))
        rec_thread.start()

        while ctx.ok():

            state = robot.get_state()
            record(state)
            t = time.time() - start_time

            if t < 2.0:
                force_command = np.zeros(6)

            elif t < act_time+2:
                force = 2.0 + (t - 2.0) * (6.0 / act_time)
                force_command = np.array([0, 0, -force, 0, 0, 0])

            elif t < 10:
                force_command = np.array([0, 0, -8, 0, 0, 0])
            
            else:
                robot.stop_controller()
                break
            
            frcController.set_control(force_command)
        
        while True:
            if data_queue.empty():
                rec_thread.join()
                break

    
    robot.move_to_joint_position(contact_pos)


def SINFORCE_run(coeff, filename):

    robot.move_to_joint_position(contact_pos)

    time.sleep(2)

    robot.start_controller(frcController)

    with robot.create_context(frequency = config['robot']['operating_freq']) as ctx:

        start_time = time.time()

        rec_thread = threading.Thread(target=recording_thread, args=(filename,))
        rec_thread.start()

        while ctx.ok():

            state = robot.get_state()
            record(state)
            t = time.time() - start_time

            if t < 1.0:
                force_command = np.zeros(6)

            elif t < 9.0:
                force = 5 + 3 * math.sin(coeff*t - coeff)
                force_command = np.array([0, 0, -force, 0, 0, 0])
            
            else:
                robot.stop_controller()
                break
            
            frcController.set_control(force_command)
        
        while True:
            if data_queue.empty():
                rec_thread.join()
                break
    
    robot.move_to_joint_position(contact_pos)
    

def SWEEP_run(coeff, filename):

    robot.move_to_joint_position(contact_pos)

    time.sleep(2)

    robot.start_controller(frcController)

    with robot.create_context(frequency = config['robot']['operating_freq']) as ctx:

        start_time = time.time()

        rec_thread = threading.Thread(target=recording_thread, args=(filename,))
        rec_thread.start()

        while ctx.ok():

            state = robot.get_state()
            record(state)
            t = time.time() - start_time

            if t < 1.0:
                force_command = np.zeros(6)

            elif t < 9.0:
                force = 5 + 3 * math.sin((coeff*t - coeff)**2)
                force_command = np.array([0, 0, -force, 0, 0, 0])
            
            else:
                robot.stop_controller()
                break
            
            frcController.set_control(force_command)
        
        while True:
            if data_queue.empty():
                rec_thread.join()
                break
    
    robot.move_to_joint_position(contact_pos)




if  __name__ == '__main__':

    for state in states:

        run(state)



    

