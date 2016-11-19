import socket
import zmq

context = zmq.Context()
outsocket = context.socket(zmq.PUB)
outsocket.setsockopt(zmq.SNDHWM, 1)
outsocket.bind("tcp://127.0.0.1:" + "5004")

UDP_IP = "127.0.0.1"
UDP_PORT = 9999
sock = socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

data = {}
while True:
    try:
        msg = sock.recv(1024)
        msg = msg.split(',')
        msg[2] = msg[2][:-1] # remove final newline
        msg[0] = (float(msg[0]) - 54)*0.0507
        msg[1] = (float(msg[1]) - 46)*0.0507
        msg[2] = (float(msg[2]) - 72)*0.3774
        data.update({'measured fx': str(msg[0]), 'measured fy': str(msg[1]),
                                                'measured fz': str(msg[2])})
        print msg
        outsocket.send_json(data)
    except KeyboardInterrupt as e:
        del sock
        break
