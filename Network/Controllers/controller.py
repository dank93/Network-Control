from __future__ import print_function
import zmq
import sys
import time
import importlib
controlfun = importlib.import_module(sys.argv[1])
outhost = str(sys.argv[2])
pubhz = float(str(sys.argv[3]))
pubperiod = 1/pubhz
localhosts = list(sys.argv[4:])

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

data = {}
receivedsocks = []
fulldata = False
t = time.time()
command = {'0':'0', '1':'0', '2':'0', '3':'0', 'control time': '0'}
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
            print(1)

        if ((fulldata == True) and  # if we've received data on all subs
                    (list(set(controlfun.datatypes) - set(data.keys())) == []) and
                    (time.time() - t > pubperiod)):
            #print (time.time() - t)
            t = time.time()
            command = controlfun.control(data) #calculate commands
            data.update(command)
            outsocket.send_json(data) # send 'em
            #print(data['l right'], data['l left'])
            data = {}
            receivedsocks = []

    except KeyboardInterrupt:
        command = {LA:'0', LB:'0', RA:'0', RB:'0'}# send dead commands, exit
        outsocket.send_json(command)
        sys.exit(0)
