#!/usr/bin/env python

"""Script to read torso mounted IMU"""
import sys
import um7
import time
import zmq

name = str(sys.argv[1])
port = str(sys.argv[2])
localhost = str(sys.argv[3])
measurements = list(sys.argv[4:])
timevar = name + ' time'

context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.setsockopt(zmq.SNDHWM, 1)
socket.bind("tcp://127.0.0.1:" + localhost)

sensor = um7.UM7(name, port, measurements, baud=921600)
time.sleep(2)
sensor.settimer()
print 'Starting BT...'
sensor.btstart()
while not sensor.zerogyros():
    pass
while not sensor.resetekf():
    pass
data = {}
try:
    print 'Taking data...'
    while True:
        while True:
            newmsg = sensor.catchsample()
            if newmsg: data.update(newmsg)
            print data
            if set(sensor.statevars) - set(data.keys()) == set([timevar]):
                break
        try:
            for i in data.keys():
                if i != timevar:
                    data[i] = data[i]*3.14159/180.0
                data[i] = str(data[i])
            socket.send_json(data)
            data = {}
        except TypeError as e:
            print('TypeError')
except KeyboardInterrupt:
    print('Done')
    del sensor
