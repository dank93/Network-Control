#!/usr/bin/env python
import zmq
import sys


inhost = str(sys.argv[1])
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.setsockopt(zmq.SUBSCRIBE, '')
socket.connect("tcp://127.0.0.1:" + inhost)

msg = socket.recv_json()
for i in sorted(msg.keys()):
    print(i)
