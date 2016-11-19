import time
import numpy as np
from math import *
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
datatypes = ['left pitch', 'left pitchrate', 'right pitch',
                'right pitchrate', 'l left', 'ddt l left', 'l right',
                'ddt l right', 'kxp', 'kyp', 'xon', 'yon', 'new eq',
                'xscale', 'yscale'] # necessary data type
x_0 = 0.0
y_0 = 0.0
def control(data): # control algorithm (return as all strings)
    global x_0, y_0
    ########## Stiffness Parameters ##########
    Kxp = float(data['kxp']) # This is total, for each side it must be divided by 2 (N/m)
    Kxd = 0.0
    kphip = 0.0
    kphid = 0.0
    Kyp = float(data['kyp']) # This is total, for each side it must be divided by 2 (N/m)
    Kyd = 200.0 # Also must be divided by 2 (N/m/s)
    Ks = 9709.0/3.0 # per leg (N/m) (not divided by 2 later)
    mExtra = 0.72

    ########## Geometry ##########
    theta_r_0 = PI/2.0
    l_0 = 0.7 + mExtra # this is the leg length at equilibrium with human weight applied
    l_unsprung = 0.14 + mExtra # this is the unspring leg length
    # x_0 = 0.0
    phi_0 = 0.0
    l_s = 0.4826 # distance between robotic shoulders
    x_right_plant = 0.4826  ##############################################################################################################

    ########### Data Extraction ##########
    phi_robot_left = float(data['left pitch']) + PI/2
    ddt_phi_robot_left = float(data['left pitchrate'])
    phi_robot_right = float(data['right pitch']) + PI/2
    ddt_phi_robot_right = float(data['right pitchrate'])
    l_left = float(data['l left'])/100  + mExtra # m
    ddt_l_left = float(data['ddt l left'])/100 # m/s
    l_right = float(data['l right'])/100 + mExtra  # m
    ddt_l_right = float(data['ddt l right'])/100 # m/s
    neweq = data['new eq']

    ########## Task-Space Geometry ##########
    xleft = l_left*math.cos(phi_robot_left)
    zleft = l_left*math.sin(phi_robot_left)
    xright = x_right_plant +  l_right*math.cos(phi_robot_right)
    zright = l_right*math.sin(phi_robot_right)
    phi_torso = atan2((zright-zleft),(xright-xleft))
    x = l_left*math.cos(phi_robot_left) + (l_s/2)*math.cos(phi_torso) - x_right_plant/2
    z = l_left*math.sin(phi_robot_left) + (l_s/2)*math.sin(phi_torso)

    ddt_phi_torso = ddt_l_left*((l_left*sin(phi_robot_left) - l_left*sin(phi_robot_right))*(-cos(phi_robot_left) + cos(phi_robot_right))/((-l_left*sin(phi_robot_left) + l_left*sin(phi_robot_right))**2 + (-l_left*cos(phi_robot_left) + l_left*cos(phi_robot_right) + x_right_plant)**2) + (-sin(phi_robot_left) + sin(phi_robot_right))*(-l_left*cos(phi_robot_left) + l_left*cos(phi_robot_right) + x_right_plant)/((-l_left*sin(phi_robot_left) + l_left*sin(phi_robot_right))**2 + (-l_left*cos(phi_robot_left) + l_left*cos(phi_robot_right) + x_right_plant)**2)) + ddt_l_right*((l_left*sin(phi_robot_left) - l_left*sin(phi_robot_right))*(-cos(phi_robot_left) + cos(phi_robot_right))/((-l_left*sin(phi_robot_left) + l_left*sin(phi_robot_right))**2 + (-l_left*cos(phi_robot_left) + l_left*cos(phi_robot_right) + x_right_plant)**2) + (-sin(phi_robot_left) + sin(phi_robot_right))*(-l_left*cos(phi_robot_left) + l_left*cos(phi_robot_right) + x_right_plant)/((-l_left*sin(phi_robot_left) + l_left*sin(phi_robot_right))**2 + (-l_left*cos(phi_robot_left) + l_left*cos(phi_robot_right) + x_right_plant)**2)) + ddt_phi_robot_left*(l_left*(l_left*sin(phi_robot_left) - l_left*sin(phi_robot_right))*sin(phi_robot_left)/((-l_left*sin(phi_robot_left) + l_left*sin(phi_robot_right))**2 + (-l_left*cos(phi_robot_left) + l_left*cos(phi_robot_right) + x_right_plant)**2) - l_left*(-l_left*cos(phi_robot_left) + l_left*cos(phi_robot_right) + x_right_plant)*cos(phi_robot_left)/((-l_left*sin(phi_robot_left) + l_left*sin(phi_robot_right))**2 + (-l_left*cos(phi_robot_left) + l_left*cos(phi_robot_right) + x_right_plant)**2)) + ddt_phi_robot_right*(-l_left*(l_left*sin(phi_robot_left) - l_left*sin(phi_robot_right))*sin(phi_robot_right)/((-l_left*sin(phi_robot_left) + l_left*sin(phi_robot_right))**2 + (-l_left*cos(phi_robot_left) + l_left*cos(phi_robot_right) + x_right_plant)**2) + l_left*(-l_left*cos(phi_robot_left) + l_left*cos(phi_robot_right) + x_right_plant)*cos(phi_robot_right)/((-l_left*sin(phi_robot_left) + l_left*sin(phi_robot_right))**2 + (-l_left*cos(phi_robot_left) + l_left*cos(phi_robot_right) + x_right_plant)**2))
    ddt_x = cos(phi_robot_left)*ddt_l_left - l_left* sin(phi_robot_left)*ddt_phi_robot_left - (l_s/2)* sin(phi_torso)*ddt_phi_torso
    ddt_z =  sin(phi_robot_left)*ddt_l_left + l_left* cos(phi_robot_left)*ddt_phi_robot_left + (l_s/2)* cos(phi_torso)*ddt_phi_torso

    ########## Equilibrium Geometry ### #######
    if neweq == '1':
        x_0 = x

    ########## Calculate Torques ##########
    F_axial = (l_unsprung - l_left)*Ks
    Fx_spring = F_axial*math.cos(phi_robot_left)
    F_goal = (x_0-x)*Kxp
    F_motor = F_goal - Fx_spring
    tau = (-F_motor*math.sin(phi_robot_left))*l_left

    #YZ PLANE
    theta_robot_left = float(data['left roll']) + PI/2 + 0.13
    ddt_theta_robot_left = float(data['left rollrate'])
    theta_robot_right = float(data['right roll']) + PI/2
    ddt_theta_robot_right = float(data['right rollrate'])

    ########## Task-Space Geometry ##########
    y = l_left*math.cos(theta_robot_left)
    ddt_y = math.cos(theta_robot_left)*ddt_l_left - l_left*math.sin(theta_robot_left)*ddt_theta_robot_left
    y_left = l_left*math.cos(theta_robot_left)
    ddt_y_left = math.cos(theta_robot_left)*ddt_l_left - l_left*math.sin(theta_robot_left)*ddt_theta_robot_left
    y_right = l_right*math.cos(theta_robot_right)
    ddt_y_right = math.cos(theta_robot_right)*ddt_l_right - l_right*math.sin(theta_robot_right)*ddt_theta_robot_right
    if neweq == '1':
        y_0 = y_left
    # print x_0

    ########## Task-Space Goal Impedance ##########
    Fy = (y_0-y_left)*Kyp + (0-ddt_y_left)*Kyd

    ########## Left Side Calcs ##########
    tau_left = (l_left*(1/(math.tan(theta_robot_left)))*Ks*(l_unsprung - l_left)
                - l_left*(1/math.sin(theta_robot_left))*(Kyp/2*(l_0*math.cos(theta_r_0) - l_left*math.cos(theta_robot_left))
                + Kyd/2*(-math.cos(theta_robot_left)*ddt_l_left + l_left*math.sin(theta_robot_left)*ddt_theta_robot_left)))

    ########## Return Commands ##########
    LAcommand = tau_left*float(data['yscale'])
    LBcommand = tau*float(data['xscale'])
    RAcommand = -tau_left*float(data['yscale'])
    RBcommand = tau*float(data['xscale'])
    if data['xon'] == '0':
        print 'off'
        LBcommand = 0
        RBcommand = 0
    if data['yon'] == '0':
        LAcommand = 0
        RAcommand = 0
    com = {LA:str(LAcommand), LB:str(LBcommand), RA:str(RAcommand),
                        RB:str(RBcommand), 'control time': str(time.time() - t0),
                        'x': str(x), 'z':str(z), 'theta robot left':str(theta_robot_left),
                        'phi': str(phi_torso), 'z right': str(zright), 'z left':str(zleft),
                        'x right':str(xright),'x left':str(xleft)}
    return com
################################################################################
################################################################################
################################################################################
