import pyqtgraph as pg
import random
import zmq
import sys
import time
import numpy as np

inhost = str(sys.argv[1])
datatypes = list(sys.argv[2:])
numtypes = len(datatypes)

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.setsockopt(zmq.SUBSCRIBE, '')
socket.setsockopt(zmq.LINGER, 0)
socket.connect("tcp://127.0.0.1:" + inhost)
# socket.disconnect("tcp://127.0.0.1:" + inhost)

app = pg.QtGui.QApplication([])

x = np.zeros((100))
y = np.zeros((numtypes,100))

pw = pg.plot()
pw.addLegend()
for num, dtype in enumerate(datatypes):
        pw.plot(x, y[num], pen=(num,numtypes), clear=False, name=dtype)

msg = {}
t0= time.time()
data = {}
while True:
        socket.connect("tcp://127.0.0.1:" + inhost)
        msg = socket.recv_json()
        socket.disconnect("tcp://127.0.0.1:" + inhost)
        for num, dtype in enumerate(datatypes):
                if dtype in msg.keys():
                        y[num] = np.roll(y[num], -1)
                        y[num,-1] = float(msg[dtype].encode('ascii', 'ignore'))
        x = np.roll(x,-1)
        x[-1] = time.time() - t0
        for num, dtype in enumerate(datatypes):
                if num == 0:
                        pw.plot(x, y[num], pen=(num,numtypes), clear=True)
                else:
                        pw.plot(x, y[num], pen=(num,numtypes), clear=False)
        app.processEvents()
