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
datatypes = ["theta left b", "ddt theta left b"] # necessary data type=
kp = 20.0 # Nm/rad
kd = 10.0 # Nm-s/rad
def control(): # control algorithm (return as all strings)
    global kp, kd
    theta = float(data["theta left b"])
    omega = float(data["ddt theta left b"])
    LBcommand = (0 - theta)*kp + (0 - omega)*kd
    com = {LA:'0', LB:str(LBcommand), RA:'0', RB:'0', 'control time': str(time.time() - t0)}
    return com
#########################################################

data = {}
receivedsocks = []
fulldata = False
t0 = time.time()
t = time.time()
command = {LA:'0', LB:'0', RA:'0', RB:'0', 'control time': str(t0)}
print("Controller Initiated")
while True:
    try:
        socks = dict(poller.poll(0)) # poll incoming connections
        receivedsocks = receivedsocks + socks.keys() # make list of subs that have new messages

        for i in insockets: # read in and store any new messages
            if i in socks:
                try:
                    msg = i.recv_json(flags=zmq.NOBLOCK)
                    data.update(msg)
                except zmq.ZMQError as e:
                    pass

        if set(insockets) - set(receivedsocks) == set([]):
            fulldata = True

        if ((fulldata == True) and  # if we've received data on all subs
                    (list(set(datatypes) - set(data.keys())) == []) and
                    (time.time() - t > pubperiod)):
            print (time.time() - t)
            t = time.time()
            command = control() #calculate commands
            data.update(command)
            outsocket.send_json(data) # send 'em
            data = {}
            receivedsocks = []

    except KeyboardInterrupt:
        command = {LA:'0', LB:'0', RA:'0', RB:'0'}# send dead commands, exit
        outsocket.send_json(command)
        sys.exit(0)
