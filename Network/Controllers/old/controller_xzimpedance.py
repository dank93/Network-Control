from __future__ import print_function
import zmq
import sys
import time
import numpy as np
import math


LA = '0'
LB = '1'
RA = '2'
RB = '3'
PI = 3.14159

outhost = str(sys.argv[1])
pubhz = float(str(sys.argv[2]))
pubperiod = 1/pubhz
inhost = str(sys.argv[3])

context = zmq.Context()
outsocket = context.socket(zmq.PUB)
outsocket.setsockopt(zmq.SNDHWM, 1)
outsocket.bind("tcp://127.0.0.1:" + outhost)
insocket = context.socket(zmq.SUB)
insocket.setsockopt(zmq.SUBSCRIBE, '')
insocket.setsockopt(zmq.RCVHWM, 1)
insocket.setsockopt(zmq.LINGER, 0)
insocket.connect("tcp://127.0.0.1:" + inhost)
insocket.disconnect("tcp://127.0.0.1:" + inhost)

t0 = time.time()
t = time.time()
data = {}
command = {LA:'0', LB:'0', RA:'0', RB:'0', 'control time': str(t0)}

#########################################################
datatypes = ['torso pitch', 'theta left b', 'theta right b', 'l left', 'l right'] # necessary data type
t = time.time()
delta_p_star = np.array([[0], [0], [0], [0]])
delta_p_star_last = np.array([[0], [0], [0], [0]])
def control(): # control algorithm (return as all strings)
    ########## Global Declarations ##########
    global t
    global delta_p_star
    global delta_p_star_last

    ########## Stiffness Parameters ##########
    Kxp = 0.0 # N/m
    Kxd = 0.0 # N/m/s
    Kmp = 0.0 # Nm/rad
    Kmd = 0.0 # Nm/rad/sec
    Ks = 0.0 # passive spring constant (N/m)
    Kp_star = np.diag([Kxp, Kmp, Ks, Ks])
    Kd_star = np.diag([Kxd, Kmd, 0.0, 0.0])

    ########## Equilibrium Geometry & Parameters ##########
    x_right_plant = 1.0 # Distance from left robot plant to right (m)
    l_s = 1.0 # shoulder-to-shoulder length (m)
    phi_left_0 = PI/2.0
    phi_torso_0 = 0.0
    l_left_0 = 1.0 #equilibrium spring length with human weight applied
    l_unsprung = 1.0 # leg length when unsprung (no weight)
    x_0 = l_left_0*math.cos(phi_left_0) + (l_s/2.0)*math.cos(phi_torso_0)

    ########### Data Extraction ##########
    phi_torso = float(data['torso pitch'])
    phi_tr_left = float(data['theta left b'])
    phi_tr_right = float(data['theta right b'])
    l_left = float(data['l left'])
    l_right = float(data['l right'])
    dt = time.time() - t
    t = time.time()

    ########## Current Geometry ##########
    phi_robot_left = phi_torso + phi_tr_left + PI
    phi_robot_right = phi_torso + phi_tr_right + PI
    x = l_left*math.cos(phi_robot_left) + (l_s/2.0)*math.cos(phi_torso)
    z = l_left*math.sin(phi_robot_left) + (l_s/2.0)*math.sin(phi_torso)

    ########## Control Calcs ##########
    # Calculate position errors, error derivatives
    delta_x = x_0 - x
    delta_phi_torso = phi_torso_0 - phi_torso
    delta_l_left = l_unsprung - l_left
    delta_l_right = l_unsprung - l_right
    delta_p_star_last = delta_p_star
    delta_p_star = np.array([[delta_x], [delta_phi_torso], [delta_l_left], [delta_l_right]])
    d_p_star_d_t = (delta_p_star - delta_p_star_last)/dt
    # Calculate Endpoint Force
    F_star = np.dot(Kp_star, delta_p_star) + np.dot(Kd_star, d_p_star_d_t)
    # Calculate Instantaneous Jacobian*
    # J_star =
    # Calculate Mixed Actuator Effort
    # tau_star = np.dot(J_star.T, F_star)

    ########## Return Commands ##########
    LAcommand = 0
    LBcommand = 0
    RAcommand = 0
    RBcommand = 0
    com = {LA:str(LAcommand), LB:str(LBcommand), RA:str(RAcommand), RB:str(RBcommand), 'control time': str(time.time() - t0)}
    return com
#########################################################

while True:
    try:
        try:
            insocket.connect("tcp://127.0.0.1:" + inhost)
            data = insocket.recv_json(flags=zmq.NOBLOCK) # grab data
            insocket.disconnect("tcp://127.0.0.1:" + inhost)
        except zmq.Again as e:
            pass

        if time.time() - t > pubperiod: # publish command
            if list(set(datatypes) - set(data.keys())) == []: # make sure all req'd datatypes are available (compare to datatypes template list)
                command = control() #calculate commands
                outsocket.send_json(command) # send 'em
                print(command)

    except KeyboardInterrupt:
        command = {LA:'0', LB:'0', RA:'0', RB:'0'}# send dead commands, exit
        outsocket.send_json(command)
        sys.exit(0)
