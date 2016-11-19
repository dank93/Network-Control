from __future__ import print_function
import zmq
import sys
import numpy as np
import time


host = str(sys.argv[1])
filename = "../Data Logs/" + str(sys.argv[2])
types = list(sys.argv[3:])

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.setsockopt(zmq.SUBSCRIBE, '')
socket.connect("tcp://127.0.0.1:" + host)

if types[0] == 'all':
    msg = socket.recv_json()
    types = msg.keys()
data = np.empty([1,len(types)+1])
log = np.empty([0,len(types)+1])

print('Logging...')
t0 = time.time()
try:
    while True:
        msg = socket.recv_json()
        data[0,0] = time.time() - t0
        for n, dtype in enumerate(types):
            if dtype in msg.keys():
                data[0,n+1] = msg[dtype].encode('ascii', 'ignore')
        log = np.vstack((log, data))
except KeyboardInterrupt as e:
        np.savetxt(filename, log, delimiter=",", header=("time,"+",".join(types)))
        print("Data Saved to " + filename + "!")
