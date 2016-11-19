import serial
import sys
import time
import zmq

port = str(sys.argv[1])
localhostout = str(sys.argv[2])
inhost = str(sys.argv[3])

context = zmq.Context() # create TCP context
socket = context.socket(zmq.PUB) # create data publishing socket
socket.setsockopt(zmq.SNDHWM,1)
socket.bind("tcp://127.0.0.1:" + localhostout)
insocket = context.socket(zmq.SUB) # create command subsriber
insocket.setsockopt(zmq.SUBSCRIBE, '')
socket.setsockopt(zmq.LINGER, 0)
socket.setsockopt(zmq.RCVHWM, 1)
insocket.connect("tcp://127.0.0.1:" + inhost)

serial = serial.Serial(port=port, baudrate=115200)#37500

arduinodatalist = ['theta left a', 'ddt theta left a','theta left b',
                'ddt theta left b','theta right a', 'ddt theta right a',
                'theta right b', 'ddt theta right b','l left', 'ddt l left',
                'l right', 'ddt l right', 'torque', 'arduino loop time',
                'arduino current time']
data = {}
command = {}
t = time.time()
msg = ""
datastream = []
newmsg = False
t0 = time.time()
msg = "0"
print("Link to Arduino Succesful")
while True:
    try: # try to read command message
        command = insocket.recv_json(flags=zmq.NOBLOCK)
        if (list(set(['0', '1', '2', '3']) - set(command.keys())) == []):
            arduinomsg = command['0'].encode('ascii', 'ignore') + "/"
            arduinomsg = arduinomsg + command['1'].encode('ascii', 'ignore') + "/"
            arduinomsg = arduinomsg + command['2'].encode('ascii', 'ignore') + "/"
            arduinomsg = arduinomsg + command['3'].encode('ascii', 'ignore') + "/"
            checksum = sum([ord(i) for i in arduinomsg])
            arduinomsg = "s" + arduinomsg + str(checksum) + "/" + "\n"
            serial.write(arduinomsg)
    except zmq.ZMQError as e: # move on if there is none
        pass

    if serial.inWaiting() > 0: # if arduino sent data
        print("Arduino message received")
        msg = serial.readline() # read it
        if msg[0] == "s": # check for new packet
            newmsg = True
            datastream = msg.split(",")[1:] #split data and get rid of start
            datastream[-1] = datastream[-1][:-2] #get rid of \r\n on last data
            for n, d in enumerate(datastream):
                try:
                    datastream[n] = str(float(d)/100)
                    if float(datastream[n]) == 999: datastream[n] = "nan"
                    data.update({arduinodatalist[n]: datastream[n]})
                except (ValueError, IndexError) as e: # suhhh
                    print ("Incoming Message Failure", d)
                    newmsg = False
            socket.send_json(data) # send to multipoller
            print("sent")

    if serial.inWaiting() > 500: # clean up
        print("flush", serial.inWaiting())
        serial.flushInput()
