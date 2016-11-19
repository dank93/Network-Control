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
                'xscale', 'yscale', 'ddt theta left a', 'ddt theta left b',
                 'ddt theta right a', 'ddt theta right b'] # necessary data type
x_0_left = 0.0
x_0_right = 0.0
y_0_left = 0.0
y_0_right = 0.0
def control(data): # control algorithm (return as all strings)
    global x_0_left, x_0_right, y_0_right, y_0_left

    ########## Stiffness Parameters ##########
    Kxp = float(data['kxp']) # This is total, for each side it must be divided by 2 (N/m)
    Kyp = float(data['kyp']) # This is total, for each side it must be divided by 2 (N/m)
    Kyd = 20.0
    Kxd = 10.0
    mExtra = 0.72

    # data extraction
    phi_robot_left = float(data['left pitch'])
    phi_robot_right = float(data['right pitch'])
    theta_robot_left = float(data['left roll'])
    theta_robot_right = float(data['right roll'])
    l_left = float(data['l left'])/100  + mExtra # m
    l_right = float(data['l right'])/100 + mExtra  # m
    neweq = data['new eq']

    # Geometry
    xleft = l_left*math.sin(phi_robot_left)
    xright = l_left*math.sin(phi_robot_right)
    yleft = l_left*math.cos(theta_robot_left)
    yright = l_left*math.cos(theta_robot_right)

    # set equilibria
    if neweq == '1':
        x_0_left = xleft
        x_0_right = xright
        y_0_left = theta_robot_left
        y_0_right = theta_robot_right

    ########## Calculate Torques ##########
    LBcommand = ((x_0_left-xleft)*Kxp - Kxd*float(data['ddt theta left b']))*float(data['xscale'])
    RBcommand = ((x_0_right-xright)*Kxp - Kxd*float(data['ddt theta left b']))*float(data['xscale'])
    LAcommand = ((y_0_left-theta_robot_left)*Kyp - Kyd*float(data['ddt theta left a']))*float(data['yscale'])
    RAcommand = (-(y_0_right-theta_robot_right)*Kyp - Kyd*float(data['ddt theta right a']))*float(data['yscale'])

    if data['xon'] == '0':
        print 'off'
        LBcommand = 0
        RBcommand = 0
    if data['yon'] == '0':
        LAcommand = 0
        RAcommand = 0
    if float(data['l left']) > 12 or float(data['l right']) > 12:
        LBcommand = 0
        RBcommand = 0
        LAcommand = 0
        RAcommand = 0
    com = {LA:str(LAcommand), LB:str(LBcommand), RA:str(RAcommand),
                        RB:str(RBcommand)}
    return com
################################################################################
################################################################################
################################################################################
