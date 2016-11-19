"""Script that remotely sets controller variables via bluetooth serial interface
and reads/stores controller data"""

from Tkinter import *
import serial
import time
import struct
import zmq

outhost = str(sys.argv[1])
pubhz = float(str(sys.argv[2]))
pubperiod = 1/pubhz
pubperiodmillis = int(pubperiod*1000)
context = zmq.Context()
socket = context.socket(zmq.PUB)
socket.bind("tcp://127.0.0.1:" + outhost)

msg = {'h1':'0', 'h2':'0', 'v1':'0', 'v2':'0'}

# GUI EVENT ACTIONS
def sendData():
    msg.update({'h1':str(h1.get())})
    msg.update({'h2':str(h2.get())})
    msg.update({'v1':str(v1.get())})
    msg.update({'v2':str(v2.get())})
    socket.send_json(msg)
    master.after(pubperiodmillis, sendData)

def buttonpress():
    pass

def close():
    master.destroy()

master = Tk()
master.after(pubperiodmillis, sendData)
master.title('Controller Instrument Panel')
button = Button(master, text='Button', width=25, command=buttonpress)
disconnectButton = Button(master, text='Disconnect', width=25, command=close)
h1 = Scale(master, from_=0, to=200, label='h1', orient=HORIZONTAL)
h2 = Scale(master, from_=0, to=200, label='h2', orient=HORIZONTAL)
v1 = Scale(master, from_=0, to=200, label='v1')
v2 = Scale(master, from_=0, to=200, label='v2')
h1.grid(row=0, column=0)
h2.grid(row=0, column=1)
v1.grid(row=1, column=0)
v2.grid(row=1, column=1)
button.grid(row=2, column=0, columnspan=4)
disconnectButton.grid(row=3, column=0, columnspan=4)

mainloop()
