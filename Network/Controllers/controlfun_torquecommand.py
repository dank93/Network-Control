import time
t0 = time.time()
LA = '0'
LB = '1'
RA = '2'
RB = '3'

################################################################################
########################## Controller Function #################################
################################################################################
datatypes = [] # necessary data types
def control(data):
    LAcommand = 0
    LBcommand = 0
    RAcommand = 0
    RBcommand = 0
    com = {LA:str(LAcommand), LB:str(LBcommand), RA:str(RAcommand),
                    RB:str(RBcommand), 'control time': str(time.time() - t0)}
    return com
################################################################################
################################################################################
################################################################################
