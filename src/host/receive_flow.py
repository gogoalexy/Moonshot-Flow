#mavlinkv1.0
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil

def retrieveFlow():
    handshake = flow_module.recv_match(type='DATA_TRANSMISSION_HANDSHAKE', blocking=True)
    if handshake is None:
        print('fail')
        sys.exit(1)

    frame = []
    for capsule in range(handshake.packets):
        subframe = flow_module.recv_match(type='ENCAPSULATED_DATA', blocking=True)
        if subframe is None:
            continue
        else:
            frame = frame + subframe.data

    return np.array(frame[:64], dtype=np.uint8).reshape(8, 8)

flow_module = mavutil.mavlink_connection('/dev/ttyACM0')
#flow_module.wait_heartbeat()
#print("Heartbeat from system (system %u component %u)" % (flow_module.target_system, flow_module.target_system))

for i in range(2):
    try:
        x = retrieveFlow()
        y = retrieveFlow()
        print(x)
        print(y)
        #show = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        #cv2.imshow('frame', show)
    except:
        continue


cv2.destroyAllWindows()
flow_module.close()
