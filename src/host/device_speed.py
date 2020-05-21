#mavlinkv1.0
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil

from timer import FPS

def retrieveFlow():
    handshake = flow_module.recv_match(type='DATA_TRANSMISSION_HANDSHAKE', blocking=True)
    if handshake is None:
        print('fail')
        sys.exit(1)


tik = FPS().start()
flow_module = mavutil.mavlink_connection('/dev/ttyACM0')
#flow_module.wait_heartbeat()
#print("Heartbeat from system (system %u component %u)" % (flow_module.target_system, flow_module.target_system))

fps = 0
localcounter = 0
while(True):
    localcounter += 1
    canvas = np.zeros((256, 256, 3))
    try:
        flow = retrieveFlow()
        index = 0
        tik.update()


        if localcounter == 10:
            tik.stop()
            fps = tik.fps()
            tik.reset()
            tik.start()
            localcounter = 0

        print(fps)

    except:
        print("err")
        continue


cv2.destroyAllWindows()
flow_module.close()
