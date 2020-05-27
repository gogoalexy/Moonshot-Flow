#mavlinkv1.0
import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np
from pymavlink import mavutil

from timer import FPS

def retrieveFrame():
    handshake = flow_module.recv_match(type='DATA_TRANSMISSION_HANDSHAKE', blocking=True)
    if handshake is None:
        print('fail')
        sys.exit(1)

    frame = []
    #print("{} packets; {} width".format(handshake.packets, handshake.width))
    for capsule in range(handshake.packets):
        subframe = flow_module.recv_match(type='ENCAPSULATED_DATA', blocking=True)
        if subframe is None:
            continue
        else:
            frame = frame + subframe.data

    return np.array(frame[:12996], dtype=np.uint8).reshape((114, 114))


flow_module = mavutil.mavlink_connection('/dev/ttyACM0')
prvs = np.zeros((114, 114), np.uint8)

tik = FPS().start()
fps = 0
localcounter = 0
while(True):
    localcounter += 1

    curr = retrieveFrame()
    tik.update()

    if localcounter >= 20:
        tik.stop()
        fps = tik.fps()
        tik.reset()
        tik.start()
        localcounter = 0

    print(fps)
    show = cv2.cvtColor(curr, cv2.COLOR_GRAY2BGR)
    show = cv2.resize(show, (640, 640))
    cv2.putText(show, str(int(fps)), (20, 20), cv2.FONT_HERSHEY_PLAIN, 0.8, (150, 150, 0))
    cv2.imshow('PX4FLOW', show)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cv2.destroyAllWindows()
flow_module.close()
