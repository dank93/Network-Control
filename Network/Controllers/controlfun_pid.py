import time
t0 = time.time()
LA = '0'
LB = '1'
RA = '2'
RB = '3'

################################################################################
########################## Controller Function #################################
################################################################################
datatypes = ["theta left b", "ddt theta left b"] # necessary data type=
kp = 20.0 # Nm/rad
kd = 10.0 # Nm-s/rad
def control(data): # control algorithm (return as all strings)
    theta = float(data["theta left b"])
    omega = float(data["ddt theta left b"])
    LAcommand = 0
    LBcommand = (0 - theta)*kp + (0 - omega)*kd
    RAcommand = 0
    RBcommand = 0
    com = {LA:str(LAcommand), LB:str(LBcommand), RA:str(RAcommand),
                    RB:str(RBcommand), 'control time': str(time.time() - t0)}
    return com
################################################################################
################################################################################
################################################################################
