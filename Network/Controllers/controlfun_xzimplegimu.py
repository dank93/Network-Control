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
                'ddt l right'] # necessary data type
def control(data): # control algorithm (return as all strings)
    ########## Stiffness Parameters ##########
    Kxp = 0.0 # This is total, for each side it must be divided by 2 (N/m)
    Kxd = 0.0
    kphip = 0.0
    kphid = 0.0
    Ks = 9709.0/3.0 # per leg (N/m) (not divided by 2 later)
    mExtra = 0.72

    ########## Equilibrium Geometry ##########
    theta_r_0 = PI/2.0
    l_0 = 0.7 + mExtra # this is the leg length at equilibrium with human weight applied
    l_unsprung = 0.14 + mExtra # this is the unspring leg length
    x_0 = 0.0
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

    ########## Task-Space Goal Impedance ##########
    Fx = (x_0-x)*Kxp
    My = (phi_0-phi_torso)*kphip

    ########## Calculate Torques ##########
    delta_p_star = np.array([[x_0-x], [phi_0-phi_torso], [l_unsprung-l_left], [l_unsprung-l_right]])
    K_p_star = np.array([[Kxp, 0, 0, 0],
                         [0, kphip, 0, 0],
                         [0, 0, Ks, 0],
                         [0, 0, 0, Ks]])
    K_d_star = np.array([[Kxd, 0, 0, 0],
                         [0, kphid, 0, 0],
                         [0, 0, Ks, 0],
                         [0, 0, 0, Ks]])
    j11 = -2*(-l_s*sin(phi_torso) + 2*z)/((-l_s*cos(phi_torso) + 2*x)**2*((-l_s*sin(phi_torso) + 2*z)**2/(-l_s*cos(phi_torso) + 2*x)**2 + 1))
    j12 = 2/((-l_s*cos(phi_torso) + 2*x)*((-l_s*sin(phi_torso) + 2*z)**2/(-l_s*cos(phi_torso) + 2*x)**2 + 1))
    j13 = (-l_s*(-l_s*sin(phi_torso) + 2*z)*sin(phi_torso)/(-l_s*cos(phi_torso) + 2*x)**2 - l_s*cos(phi_torso)/(-l_s*cos(phi_torso) + 2*x))/((-l_s*sin(phi_torso) + 2*z)**2/(-l_s*cos(phi_torso) + 2*x)**2 + 1)
    j21 = -(-l_s*sin(phi_torso) + 2*z)**2/((-l_s*cos(phi_torso) + 2*x)**2*sqrt((-l_s*sin(phi_torso) + 2*z)**2/(-l_s*cos(phi_torso) + 2*x)**2 + 1)) + sqrt((-l_s*sin(phi_torso) + 2*z)**2/(-l_s*cos(phi_torso) + 2*x)**2 + 1)
    j22 = (-4*l_s*sin(phi_torso) + 8*z)/(4*(-l_s*cos(phi_torso) + 2*x)*sqrt((-l_s*sin(phi_torso) + 2*z)**2/(-l_s*cos(phi_torso) + 2*x)**2 + 1))
    j23 = l_s*sqrt((-l_s*sin(phi_torso) + 2*z)**2/(-l_s*cos(phi_torso) + 2*x)**2 + 1)*sin(phi_torso)/2 + (-l_s*cos(phi_torso) + 2*x)*(-l_s*(-l_s*sin(phi_torso) + 2*z)**2*sin(phi_torso)/(-l_s*cos(phi_torso) + 2*x)**3 - l_s*(-l_s*sin(phi_torso) + 2*z)*cos(phi_torso)/(-l_s*cos(phi_torso) + 2*x)**2)/(2*sqrt((-l_s*sin(phi_torso) + 2*z)**2/(-l_s*cos(phi_torso) + 2*x)**2 + 1))
    j31 = -2*(l_s*sin(phi_torso) + 2*z)/((l_s*cos(phi_torso) + 2*x)**2*((x_right_plant - (l_s*sin(phi_torso) + 2*z)/(l_s*cos(phi_torso) + 2*x))**2 + 1))
    j32 = 2/((l_s*cos(phi_torso) + 2*x)*((x_right_plant - (l_s*sin(phi_torso) + 2*z)/(l_s*cos(phi_torso) + 2*x))**2 + 1))
    j33 = -(-l_s*(l_s*sin(phi_torso) + 2*z)*sin(phi_torso)/(l_s*cos(phi_torso) + 2*x)**2 - l_s*cos(phi_torso)/(l_s*cos(phi_torso) + 2*x))/((x_right_plant - (l_s*sin(phi_torso) + 2*z)/(l_s*cos(phi_torso) + 2*x))**2 + 1)
    j41 = -(-l_s*sin(phi_torso) + 2*z)*(l_s*sin(phi_torso) + 2*z)/((l_s*cos(phi_torso) + 2*x)**2*sqrt((x_right_plant - (l_s*sin(phi_torso) + 2*z)/(l_s*cos(phi_torso) + 2*x))**2 + 1)) + (-l_s*sin(phi_torso) + 2*z)*(l_s*sin(phi_torso) + 2*z)*sqrt((x_right_plant - (l_s*sin(phi_torso) + 2*z)/(l_s*cos(phi_torso) + 2*x))**2 + 1)/((x_right_plant - (l_s*sin(phi_torso) + 2*z)/(l_s*cos(phi_torso) + 2*x))**2*(l_s*cos(phi_torso) + 2*x)**2)
    j42 = (-l_s*sin(phi_torso) + 2*z)/((l_s*cos(phi_torso) + 2*x)*sqrt((x_right_plant - (l_s*sin(phi_torso) + 2*z)/(l_s*cos(phi_torso) + 2*x))**2 + 1)) - sqrt((x_right_plant - (l_s*sin(phi_torso) + 2*z)/(l_s*cos(phi_torso) + 2*x))**2 + 1)/(x_right_plant - (l_s*sin(phi_torso) + 2*z)/(l_s*cos(phi_torso) + 2*x)) - (-l_s*sin(phi_torso) + 2*z)*sqrt((x_right_plant - (l_s*sin(phi_torso) + 2*z)/(l_s*cos(phi_torso) + 2*x))**2 + 1)/((x_right_plant - (l_s*sin(phi_torso) + 2*z)/(l_s*cos(phi_torso) + 2*x))**2*(l_s*cos(phi_torso) + 2*x))
    j43 = l_s*sqrt((x_right_plant - (l_s*sin(phi_torso) + 2*z)/(l_s*cos(phi_torso) + 2*x))**2 + 1)*cos(phi_torso)/(2*(x_right_plant - (l_s*sin(phi_torso) + 2*z)/(l_s*cos(phi_torso) + 2*x))) - (-l_s*sin(phi_torso) + 2*z)*(-2*l_s*(l_s*sin(phi_torso) + 2*z)*sin(phi_torso)/(l_s*cos(phi_torso) + 2*x)**2 - 2*l_s*cos(phi_torso)/(l_s*cos(phi_torso) + 2*x))/(4*sqrt((x_right_plant - (l_s*sin(phi_torso) + 2*z)/(l_s*cos(phi_torso) + 2*x))**2 + 1)) - (-l_s*sin(phi_torso) + 2*z)*(l_s*(l_s*sin(phi_torso) + 2*z)*sin(phi_torso)/(l_s*cos(phi_torso) + 2*x)**2 + l_s*cos(phi_torso)/(l_s*cos(phi_torso) + 2*x))*sqrt((x_right_plant - (l_s*sin(phi_torso) + 2*z)/(l_s*cos(phi_torso) + 2*x))**2 + 1)/(2*(x_right_plant - (l_s*sin(phi_torso) + 2*z)/(l_s*cos(phi_torso) + 2*x))**2)
    Jstar11 = j11*j12*j33/(j11**2*j33 - j11*j13*j31) - j11*j13*j32/(j11**2*j33 - j11*j13*j31)
    Jstar12 = j11**2*j32/(j11**2*j33 - j11*j13*j31) - j11*j12*j31/(j11**2*j33 - j11*j13*j31)
    Jstar13 = -j11**2*j22*j33/(j11**2*j33 - j11*j13*j31) + j11**2*j23*j32/(j11**2*j33 - j11*j13*j31) + j11*j12*j21*j33/(j11**2*j33 - j11*j13*j31) - j11*j12*j23*j31/(j11**2*j33 - j11*j13*j31) - j11*j13*j21*j32/(j11**2*j33 - j11*j13*j31) + j11*j13*j22*j31/(j11**2*j33 - j11*j13*j31)
    Jstar14 = j11**2*j32*j43/(j11**2*j33 - j11*j13*j31) - j11**2*j33*j42/(j11**2*j33 - j11*j13*j31) - j11*j12*j31*j43/(j11**2*j33 - j11*j13*j31) + j11*j12*j33*j41/(j11**2*j33 - j11*j13*j31) + j11*j13*j31*j42/(j11**2*j33 - j11*j13*j31) - j11*j13*j32*j41/(j11**2*j33 - j11*j13*j31)
    Jstar21 = -j33/(j11*j33 - j13*j31)
    Jstar22 = j31/(j11*j33 - j13*j31)
    Jstar23 = -j21*j33/(j11*j33 - j13*j31) + j23*j31/(j11*j33 - j13*j31)
    Jstar24 = j31*j43/(j11*j33 - j13*j31) - j33*j41/(j11*j33 - j13*j31)
    J_star_T = np.array([[Jstar11, Jstar12, Jstar13, Jstar14], [Jstar21, Jstar22, Jstar23, Jstar24]])
    tau_star = np.dot(J_star_T, np.dot(K_p_star, delta_p_star))
    tau_left = tau_star[0,0]
    tau_right = tau_star[1,0]

    ########## Return Commands ##########
    LAcommand = 0
    LBcommand = tau_left
    RAcommand = 0
    RBcommand = tau_right
    com = {LA:str(LAcommand), LB:str(LBcommand), RA:str(RAcommand),
                        RB:str(RBcommand), 'control time': str(time.time() - t0),
                        'desired fx': str(Fx), 'desired My': str(My), 'x': str(x), 'z':str(z),
                        'phi': str(phi_torso), 'z right': str(zright), 'z left':str(zleft),
                        'x right':str(xright),'x left':str(xleft)}
    return com
################################################################################
################################################################################
################################################################################
