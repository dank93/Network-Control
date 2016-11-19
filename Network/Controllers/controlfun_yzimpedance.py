import time
import numpy as np
import math
PI = np.pi
t0 = time.time()
LA = '0'
LB = '1'
RA = '2'
RB = '3'

################################################################################
########################## Controller Function #################################
################################################################################
datatypes = ['torso roll', 'theta left a', 'theta right a', 'l left', 'l right'] # necessary data type
def control(data): # control algorithm (return as all strings)
    ########## Stiffness Parameters ##########
    Kyp = 0.0 # This is total, for each side it must be divided by 2 (N/m)
    Kyd = 0.0 # Also must be divided by 2 (N/m/s)
    Ks = 9709.0/3.0 # spring constant of individual passive leg springs (N/m)
    mExtra = 0.676275
    KP_star = np.diag([Kyp/2.0, Ks])
    KD_star = np.diag([Kyd/2.0, 0.0])

    ########## Equilibrium Geometry ##########
    theta_r_0 = PI/2.0
    l_0 = 0.2 + mExtra # this is the leg length at equilibrium with human weight applied
    l_unsprung = 0.2 + mExtra # this is the unspring leg length
    y_0 = l_0 * math.cos(theta_r_0)

    ########### Data Extraction ##########
    theta_torso = float(data['torso roll']) # rad
    ddt_theta_torso = float(data['torso rollrate']) # rad/sec
    theta_tr_left = float(data['theta left a']) # rad
    ddt_theta_tr_left = float(data['ddt theta left a']) # rad/sec
    theta_tr_right = -float(data['theta right a']) # rad
    ddt_theta_tr_right = -float(data['ddt theta right a']) # rad/sec
    l_left = float(data['l left'])/100  + mExtra # m
    ddt_l_left = float(data['ddt l left'])/100 # m/s
    l_right = float(data['l right'])/100 + mExtra # m
    ddt_l_right = float(data['ddt l right'])/100 # m/s

    ########## Current Geometry ##########
    theta_robot_left = theta_torso + theta_tr_left + PI/2
    ddt_theta_robot_left = ddt_theta_torso + ddt_theta_tr_left
    theta_robot_right = theta_torso + theta_tr_right + PI/2
    ddt_theta_robot_right = ddt_theta_torso + ddt_theta_tr_right

    y_left = l_left*math.cos(theta_robot_left)
    ddt_y_left = math.cos(theta_robot_left)*ddt_l_left - l_left*math.sin(theta_robot_left)*ddt_theta_robot_left
    y_right = l_right*math.cos(theta_robot_right)
    ddt_y_right = math.cos(theta_robot_right)*ddt_l_right - l_right*math.sin(theta_robot_right)*ddt_theta_robot_right
    print(theta_robot_left*180/3.14, theta_robot_right*180/3.14)

    ########## Left Side Calcs ##########
            # Calculate position errors, error derivatives
    delta_y_left = y_0 - y_left
    delta_l_left = l_unsprung - l_left
    delta_p_star_left = np.array([[delta_y_left], [delta_l_left]])
    d_p_star_left_d_t = np.array([[ddt_y_left], [ddt_l_left]])
            # Calculate Endpoint Force
    F_star_left = np.dot(KP_star, delta_p_star_left) + np.dot(KD_star, d_p_star_left_d_t)
            # Calculate Instantaneous Jacobian*
    J_star_left_11 = -l_left/math.sin(theta_robot_left)
    J_star_left_12 = -l_left/math.tan(theta_robot_left)
    J_star_left_21 = 1.0/math.tan(theta_robot_left)
    J_star_left_22 = 1.0/math.sin(theta_robot_left)
    J_star_left = np.array([[J_star_left_11, J_star_left_12],[J_star_left_21, J_star_left_22]])
            # Calculate Mixed Actuator Effort
    tau_star_left = np.dot(J_star_left.T, F_star_left)
    tau_left = tau_star_left[0,0]
    F_z_left = tau_star_left[1,0]

    ########## right Side Calcs ##########
            # Calculate position errors, error derivatives
    delta_y_right = y_0 - y_right
    delta_l_right = delta_l_left #l_unsprung - l_right
    delta_p_star_right = np.array([[delta_y_right], [delta_l_right]])
    d_p_star_right_d_t = np.array([[ddt_y_right], [ddt_l_right]])
            # Calculate Endpoint Force
    F_star_right = np.dot(KP_star, delta_p_star_right) + np.dot(KD_star, d_p_star_right_d_t)
    #        Calculate Instantaneous Jacobian
    J_star_right_11 = -l_right/math.sin(theta_robot_right)
    J_star_right_12 = -l_right/math.tan(theta_robot_right)
    J_star_right_21 = 1.0/math.tan(theta_robot_right)
    J_star_right_22 = 1.0/math.sin(theta_robot_right)
    J_star_right = np.array([[J_star_right_11, J_star_right_12],[J_star_right_21, J_star_right_22]])
            # Calculate Mixed Actuator Effort
    tau_star_right = np.dot(J_star_right.T, F_star_right)
    tau_right = -tau_star_right[0,0]
    F_z_right = tau_star_right[1,0]

    ########## Return Commands ##########
    LAcommand = tau_left #make sure orientations are right
    LBcommand = 0
    RAcommand = tau_right # make sure orientations are right
    RBcommand = 0
    com = {LA:str(LAcommand), LB:str(LBcommand), RA:str(RAcommand),
                        RB:str(RBcommand), 'control time': str(time.time() - t0),
                        'theta robot left': str(theta_robot_left),
                        'theta robot right': str(theta_robot_right),
                        'fz left': str(F_z_left),
                        'fz right': str(F_z_right)}
    return com
################################################################################
################################################################################
################################################################################
