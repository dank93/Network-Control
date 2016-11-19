from __future__ import print_function
import zmq
import sys
import time
import math


LA = '0'
LB = '1'
RA = '2'
RB = '3'

outhost = str(sys.argv[1])
pubhz = float(str(sys.argv[2]))
pubperiod = 1/pubhz
inhost = str(sys.argv[3])

context = zmq.Context()
outsocket = context.socket(zmq.PUB)
outsocket.bind("tcp://127.0.0.1:" + outhost)
insocket = context.socket(zmq.SUB)
insocket.setsockopt(zmq.SUBSCRIBE, '')
insocket.connect("tcp://127.0.0.1:" + inhost)

t0 = time.time()
t = time.time()
data = {}
command = {LA:'0', LB:'0', RA:'0', RB:'0', 'control time': str(t0)}

#########################################################
datatypes = ['torso roll'] # necessary data type
y = 0.0
def control(): # control algorithm (return as all strings)
    global y
    y0 = 0.0
    dydt0 = 0.0
    lasty = y
    l = 0.79
    m = 5.4
    kp = 40.0
    kd = 0.0
    dt = pubperiod
    g = 9.81

    theta = float(data['torso roll'])*3.14/180

    y = -l*math.sin(theta)
    dydt = (y - lasty)/dt
    deltay = y0 - y
    deltadydt = dydt0 - dydt
    fy = kp*deltay + kd*deltadydt

    tau = fy*l*math.cos(theta) + m*g*l*math.sin(theta)

    LAcommand = -tau/2.0
    RAcommand = tau/2.0
    LBcommand = 0
    RBcommand = 0

    com = {'theta':str(theta), 'y':str(y), LA:str(LAcommand), LB:str(LBcommand), RA:str(RAcommand), RB:str(RBcommand), 'control time': str(time.time() - t0)}
    return com
#########################################################

while True:
    try:
        try:
            data = insocket.recv_json(flags=zmq.NOBLOCK) # grab data
        except zmq.Again as e:
            pass

        if time.time() - t > pubperiod: # publish command
            t = time.time()
            if list(set(datatypes) - set(data.keys())) == []: # make sure all req'd datatypes are available (compare to datatypes template list)
                command = control() #calculate commands
                outsocket.send_json(command) # send 'em
                print(command)

    except KeyboardInterrupt:
        command.update({LA:'0', LB:'0', RA:'0', RB:'0'})# send dead commands, exit
        outsocket.send_json(command)
        sys.exit(0)
