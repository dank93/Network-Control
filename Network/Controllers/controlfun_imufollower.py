import time
t0 = time.time()
LA = '0'
LB = '1'
RA = '2'
RB = '3'

################################################################################
########################## Controller Function #################################
################################################################################
datatypes = ['arm roll', 'arm pitch', 'theta left a', 'ddt theta left a',
            'theta left b', 'ddt theta left b','theta right a', 'ddt theta right a',
            'theta right b', 'ddt theta right b'] # necessary data type
def control(data): # control algorithm (return as all strings)
    kp = 40.0 # set PD gains (10 works)
    kd = 10.0 #(2 works
    arefleft = float(data['arm roll'])
    brefleft = -float(data['arm pitch'])
    arefright = -arefleft
    brefright = -brefleft
    thetalefta = float(data['theta left a']) # Not sure if these orientations are right...
    thetaleftb = float(data['theta left b'])
    thetarighta = float(data['theta right a'])
    thetarightb = float(data['theta right b'])
    omegalefta = float(data['ddt theta left a'])
    omegaleftb = float(data['ddt theta left b'])
    omegarighta = float(data['ddt theta right a'])
    omegarightb = float(data['ddt theta right b'])

    thetaleftaERROR = arefleft - thetalefta
    thetaleftbERROR = brefleft - thetaleftb
    thetarightaERROR = arefright - thetarighta
    thetarightbERROR = brefright - thetarightb
    omegaleftaERROR = 0 - omegalefta
    omegaleftbERROR = 0 - omegaleftb
    omegarightaERROR = 0 - omegarighta
    omegarightbERROR = 0 - omegarightb

    LAcommand = kp*thetaleftaERROR + kd*omegaleftaERROR
    LBcommand = kp*thetaleftbERROR + kd*omegaleftbERROR
    RAcommand = kp*thetarightaERROR + kd*omegarightaERROR
    RBcommand = kp*thetarightbERROR + kd*omegarightbERROR

    com = {LA:str(LAcommand), LB:str(LBcommand), RA:str(RAcommand),
                    RB:str(RBcommand), 'control time': str(time.time() - t0)}
    return com
################################################################################
################################################################################
################################################################################
