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
datatypes = ['left roll', 'left rollrate', 'right roll',
                'right rollrate', 'l left', 'ddt l left', 'l right',
                'ddt l right'] # necessary data type
def control(data): # control algorithm (return as all strings)
    global leftroll, rightroll
    ########## Stiffness Parameters ##########
    Kyp = 0.0 # This is total, for each side it must be divided by 2 (N/m)
    Kyd = 200.0 # Also must be divided by 2 (N/m/s)
    Ks = 9709.0/3.0 # per leg (N/m) (not divided by 2 later)
    mExtra = 0.72

    ########## Equilibrium Geometry ##########
    theta_r_0 = PI/2.0
    l_0 = 0.7 + mExtra # this is the leg length at equilibrium with human weight applied
    l_unsprung = 0.14 + mExtra # this is the unspring leg length
    y_0 = l_0 * math.cos(theta_r_0)
    x_0 = 0.0
    phi_0 = 0.0
    l_s = 0.4826 # distance between robotic shoulders
    x_right_plant = 0.4826 ##############################################################################################################


    ########### Data Extraction ##########
    phi_robot_left = float(data['left pitch']) + PI/2
    ddt_phi_robot_left = float(data['left pitchrate'])
    phi_robot_right = float(data['right pitch']) + PI/2
    ddt_phi_robot_right = float(data['right pitchrate'])
    l_left = float(data['l left'])/100  + mExtra # m
    ddt_l_left = float(data['ddt l left'])/100 # m/s
    l_right = float(data['l right'])/100 + mExtra # m
    ddt_l_right = float(data['ddt l right'])/100 # m/s
    theta_robot_left = float(data['left roll']) + PI/2 +0.13
    ddt_theta_robot_left = float(data['left rollrate'])
    theta_robot_right = float(data['right roll']) + PI/2
    ddt_theta_robot_right = float(data['right rollrate'])

    ########## Task-Space Geometry ##########
    xleft = l_left*math.cos(phi_robot_left)
    zleft = l_left*math.sin(phi_robot_left)
    xright = x_right_plant +  l_right*math.cos(phi_robot_right)
    zright = l_right*math.sin(phi_robot_right)
    phi_torso = math.atan2((zright-zleft),(xright-xleft))
    x = l_left*math.cos(phi_robot_left) + (l_s/2)*math.cos(phi_torso) - x_right_plant/2
    y = l_left*math.cos(theta_robot_left)
    ddt_y = math.cos(theta_robot_left)*ddt_l_left - l_left*math.sin(theta_robot_left)*ddt_theta_robot_left
    z = l_left*math.sin(theta_robot_left)
    y_left = l_left*math.cos(theta_robot_left)
    ddt_y_left = math.cos(theta_robot_left)*ddt_l_left - l_left*math.sin(theta_robot_left)*ddt_theta_robot_left
    y_right = l_right*math.cos(theta_robot_right)
    ddt_y_right = math.cos(theta_robot_right)*ddt_l_right - l_right*math.sin(theta_robot_right)*ddt_theta_robot_right
    z = l_left*math.sin(theta_robot_left)

    ########## Task-Space Goal Impedance ##########
    Fy = (0-y_left)*Kyp + (0-ddt_y_left)*Kyd

    ########## Left Side Calcs ##########
    tau_left = (l_left*(1/(math.tan(theta_robot_left)))*Ks*(l_unsprung - l_left)
                - l_left*(1/math.sin(theta_robot_left))*(Kyp/2*(l_0*math.cos(theta_r_0) - l_left*math.cos(theta_robot_left))
                + Kyd/2*(-math.cos(theta_robot_left)*ddt_l_left + l_left*math.sin(theta_robot_left)*ddt_theta_robot_left)))

    ########## right Side Calcs ##########
    tau_right = (l_left*(1/(math.tan(theta_robot_right)))*Ks*(l_unsprung - l_left)
                - l_left*(1/math.sin(theta_robot_right))*(Kyp/2*(l_0*math.cos(theta_r_0) - l_left*math.cos(theta_robot_right))
                + Kyd/2*(-math.cos(theta_robot_right)*ddt_l_left + l_left*math.sin(theta_robot_right)*ddt_theta_robot_right)))

    predicted_fy = -Ks*(l_unsprung - l_left)*math.cos(theta_robot_left)

    ########## Return Commands ##########
    LAcommand = tau_left #make sure orientations are right
    LBcommand = 0
    RAcommand = -tau_left # make sure orientations are right
    RBcommand = 0
    com = {LA:str(LAcommand), LB:str(LBcommand), RA:str(RAcommand),
                        RB:str(RBcommand), 'control time': str(time.time() - t0),
                        'desired fy': str(Fy), 'y': str(y_left), 'z':str(z), 'ddt y':str(ddt_y_left),
                        'x': str(x), "predicted fy": str(predicted_fy),'phi torso': str(phi_torso)}
    return com
################################################################################
################################################################################
################################################################################
