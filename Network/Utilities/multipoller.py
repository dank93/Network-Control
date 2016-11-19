from __future__ import print_function
import zmq
import sys
import time

outhost = str(sys.argv[1])
pubhz = float(str(sys.argv[2]))
pubperiod = 1/pubhz
localhosts = list(sys.argv[3:])

if __name__ == "__main__":

    context = zmq.Context()
    outsocket = context.socket(zmq.PUB)
    outsocket.setsockopt(zmq.SNDHWM,1)
    outsocket.bind("tcp://127.0.0.1:" + outhost)

    sockets = []
    for i in range(len(localhosts)):
        sockets.append(context.socket(zmq.SUB))
        sockets[i].setsockopt(zmq.SUBSCRIBE, '')
        sockets[i].setsockopt(zmq.RCVHWM, 1)
        sockets[i].setsockopt(zmq.LINGER, 0)
        sockets[i].connect("tcp://127.0.0.1:" + localhosts[i])

    poller = zmq.Poller()
    for i in sockets:
        poller.register(i, zmq.POLLIN)

    t = time.time()
    data = {}
    receivedsocks = []
    fulldata = False
    while True:
        try:
            socks = dict(poller.poll())
        except KeyboardInterrupt:
            break
        receivedsocks = receivedsocks + socks.keys()

        for i in sockets:
            if i in socks:
                msg = i.recv_json()
                data.update(msg)

        if set(sockets) - set(receivedsocks) == set([]):
            fulldata = True

        dt = time.time() - t
        if fulldata == True:
            t = time.time()
            print(dt)
            outsocket.send_json(data)
            data = {}
            receivedsocks = []
            fulldata = False
