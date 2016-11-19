from __future__ import print_function
import zmq
import sys

host = str(sys.argv[1])
types = list(sys.argv[2:])

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.setsockopt(zmq.SUBSCRIBE, '')
socket.connect("tcp://127.0.0.1:" + host)

data = {}
while True:
    msg = socket.recv_json()
    for i in types:
        if i in msg.keys():
            data.update({i: msg[i]})
    print(data)
    

