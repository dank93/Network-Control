from __future__ import print_function
import zmq
import sys
import time


LA = '0'
LB = '1'
RA = '2'
RB = '3'

outhost = str(sys.argv[1])
pubhz = float(str(sys.argv[2]))
pubperiod = 1/pubhz
localhosts = list(sys.argv[3:])

context = zmq.Context()
outsocket = context.socket(zmq.PUB)
outsocket.bind("tcp://127.0.0.1:" + outhost)

insockets = []
for i in range(len(localhosts)):
    insockets.append(context.socket(zmq.SUB))
    insockets[i].setsockopt(zmq.SUBSCRIBE, '')
    insockets[i].setsockopt(zmq.RCVHWM, 1)
    insockets[i].setsockopt(zmq.LINGER, 0)
    insockets[i].connect("tcp://127.0.0.1:" + localhosts[i])

poller = zmq.Poller()
for i in insockets:
    poller.register(i, zmq.POLLIN)

#########################################################
datatypes = ['torso roll', 'theta left a', 'theta right a', 'l left', 'l right'] # necessary data type
t = time.time()
delta_p_star_left = np.array([[0], [0]])
delta_p_star_right = np.array([[0], [0]])
delta_p_star_left_last = np.array([[0], [0]])
delta_p_star_right_last = np.array([[0], [0]])
def control(): # control algorithm (return as all strings)
    ########## Global Declarations ##########
    global t
    global delta_p_star_left, delta_p_star_right
    global delta_p_star_left_last, delta_p_star_right_last

    ########## Stiffness Parameters ##########
    Kyp = 10.0 # This is total, for each side it must be divided by 2 (N/m)
    Kyd = 2.0 # Also must be divided by 2 (N/m/s)
    Ks = 1.0 # spring constant of individual passive leg springs (N/m)
    KP_star = np.diag([Kyp/2.0, Ks])
    KD_star = np.diag([Kyd/2.0, 0.0])

    ########## Equilibrium Geometry ##########
    theta_r_0 = PI/2.0
    l_0 = 1.0 # this is the leg length at equilibrium with human weight applied
    l_unsprung = 1.0 # this is the unspring leg length
    y_0 = l_0 * math.cos(theta_r_0)

    ########### Data Extraction ##########
    theta_torso = float(data['torso roll'])
    theta_tr_left = float(data['theta left a']) # make sure orientations are correct
    theta_tr_right = -float(data['theta right a']) # same as comment above
    l_left = float(data['l left']) # make sure dimensions are correct
    l_right = float(data['l right']) # same comment as above
    dt = time.time() - t
    t = time.time()

    ########## Current Geometry ##########
    theta_robot_left = theta_torso + theta_tr_left + PI
    theta_robot_right = theta_torso + theta_tr_right + PI
    y_left = l_left*math.cos(theta_robot_left)
    y_right = l_right*math.cos(theta_robot_right)

    ########## Left Side Calcs ##########
    # Calculate position errors, error derivatives
    delta_y_left = y_0 - y_left
    delta_l_left = l_unsprung - l_left
    delta_p_star_left_last = delta_p_star_left
    delta_p_star_left = np.array([[delta_y_left], [delta_l_left]])
    d_p_star_left_d_t = (delta_p_star_left - delta_p_star_left_last)/dt
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
    tau_left = tau_star_left[1,1]
    F_z_left = tau_star_left[2,1]

    ########## right Side Calcs ##########
    # Calculate position errors, error derivatives
    delta_y_right = y_0 - y_right
    delta_l_right = l_unsprung - l_right
    delta_p_star_right_last = delta_p_star_right
    delta_p_star_right = np.array([[delta_y_right], [delta_l_right]])
    d_p_star_right_d_t = (delta_p_star_right - delta_p_star_right_last)/dt
    # Calculate Endpoint Force
    F_star_right = np.dot(KP_star, delta_p_star_right) + np.dot(KD_star, d_p_star_right_d_t)
    # Calculate Instantaneous Jacobian
    J_star_right_11 = -l_right/math.sin(theta_robot_right)
    J_star_right_12 = -l_right/math.tan(theta_robot_right)
    J_star_right_21 = 1.0/math.tan(theta_robot_right)
    J_star_right_22 = 1.0/math.sin(theta_robot_right)
    J_star_right = np.array([[J_star_right_11, J_star_right_12],[J_star_right_21, J_star_right_22]])
    # Calculate Mixed Actuator Effort
    tau_star_right = np.dot(J_star_right.T, F_star_right)
    tau_right = tau_star_right[1,1]
    F_z_right = tau_star_right[2,1]

    ########## Return Commands ##########
    LAcommand = tau_left #make sure orientations are right
    LBcommand = 0
    RAcommand = tau_right # make sure orientations are right
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
