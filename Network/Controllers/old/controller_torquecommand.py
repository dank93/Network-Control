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
datatypes = ['h1'] # necessary data type
dt = pubperiod # time step
def control(): # control algorithm (return as all strings)
    com = {LA:data['v1'], LB:data['h1'], RA:data['v2'], RB:data['h2'], 'control time': str(time.time() - t0)}
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
                print(command, data)
            
    except KeyboardInterrupt:
        command = {LA:'0', LB:'0', RA:'0', RB:'0'}# send dead commands, exit
        outsocket.send_json(command)
        sys.exit(0)
        
