"""Script that remotely sets controller variables via bluetooth serial interface
and reads/stores controller data"""

from Tkinter import *
import serial
import time
import struct
import zmq
import threading
import Queue
# from multiprocessing import Process, Queue, freeze_support
q = Queue.Queue()
# q = Queue()

outhost = str(sys.argv[1])
pubhz = float(str(sys.argv[2]))
pubperiod = 1/pubhz
pubperiodmillis = int(pubperiod*1000)

msg = {'kxp':'0', 'kyp':'0', 'xon':'0', 'yon':'0', 'new eq':'0'}
xonstate = '0'
yonstate = '0'
neweq = '0'
numeq = 0

# TCP ACTIONS
def tcpfunc(q):
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    socket.bind("tcp://127.0.0.1:" + outhost)
    msg = ''
    try:
        while True:
            try:
                msg = q.get(False)
            except Queue.Empty:
                pass
            print msg
            if msg == 'disconnect':
                socket.close()
                sys.exit(0)
            else:
                socket.send_json(msg)
    except:
        socket.close()
        sys.exit(0)

# GUI EVENT ACTIONS
def guifunc(q):
    global pubperiodmillis
    msg = {'kxp':'0', 'kyp':'0', 'xon':'0', 'yon':'0', 'new eq':'0', 'xscale':'0', 'yscale':'0'}
    xonstate = '0'
    yonstate = '0'
    neweq = '0'
    numeq = 0

    def sendData():
        global xonstate, yonstate, neweq
        msg = {}
        msg.update({'kxp':str(kxp.get())})
        msg.update({'kyp':str(kyp.get())})
        msg.update({'xon':xonstate})
        msg.update({'yon':yonstate})
        msg.update({'new eq':neweq})
        msg.update({'xscale':str(xscale.get()/100.0)})
        msg.update({'yscale':str(yscale.get()/100.0)})
        q.put(msg)
        neweq = '0'
        master.after(pubperiodmillis, sendData)

    def eqbuttonpress():
        global neweq, numeq
        neweq='1'
        numeq = numeq+1
        eqbutton["text"]="Eq: " + str(numeq)

    def xonbuttonpress():
        global xonstate
        if xon.config('relief')[-1] == 'sunken':
            xon['text'] = 'x off'
            xon.config(relief="raised")
            xon.config(bg="red")
            xonstate = '0'
        else:
            xon['text'] = 'x on'
            xon.config(relief="sunken")
            xon.config(bg="green")
            xonstate = '1'

    def yonbuttonpress():
        global yonstate
        if yon.config('relief')[-1] == 'sunken':
            yon['text'] = 'y off'
            yon.config(relief="raised")
            yon.config(bg="red")
            yonstate = '0'
        else:
            yon['text'] = 'y on'
            yon.config(relief="sunken")
            yon.config(bg="green")
            yonstate = '1'

    def close():
        q.put('disconnect')
        master.destroy()

    master = Tk()
    master.after(pubperiodmillis, sendData)
    master.title('Controller Instrument Panel')
    eqbutton = Button(master, text='Set Equilibrium', width=25, command=eqbuttonpress)
    disconnectButton = Button(master, text='Disconnect', width=25, command=close)
    xon = Button(master, text='x off', width=10, command=xonbuttonpress, relief = 'raised', bg='red')
    yon = Button(master, text='y off', width=10, command=yonbuttonpress, relief = 'raised', bg='red')
    kxp = Scale(master, from_=0, to=700, label='kxp', orient=HORIZONTAL, length = 500)
    kyp = Scale(master, from_=0, to=400, label='kyp', length = 500)
    xscale = Scale(master, from_=0, to=100, label='xscale', orient=HORIZONTAL, length = 500)
    yscale = Scale(master, from_=0, to=100, label='yscale', length = 500)
    kxp.grid(row=0, column=0)
    kyp.grid(row=1, column=0)
    xscale.grid(row=0, column=1)
    yscale.grid(row=1, column=1)
    xon.grid(row=0, column=2)
    yon.grid(row=1, column=2)
    eqbutton.grid(row=2, column=0, columnspan=4)
    disconnectButton.grid(row=3, column=0, columnspan=4)

    try:
        mainloop()
    except KeyboardInterrupt:
        q.put('disconnect')

thread1 = threading.Thread(target=tcpfunc,args=(q,))
thread2 = threading.Thread(target=guifunc,args=(q,))
thread1.start()
thread2.start()
