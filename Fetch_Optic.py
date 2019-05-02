from opticalflow import Optical_Flow
import time as t
from threading import Thread

# Start thread in the main code and call the fetch_optic
# function whenever an updated position is required

optic = Optical_Flow()  # Initiate Thread

def optic_setup():
    optic.start() # Start thread
    t.sleep(2)

def fetch():
    location=optic.position()
    
    return location

def reset():
    reset_optic=optic.reset()
    pass

'''
optic.start()
t.sleep(2)
while 1:
    print(optic.position())
    t.sleep(0.2)
''' 
