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

    frame = []
    #print("{} packets; {} width".format(handshake.packets, handshake.width))
    for capsule in range(handshake.packets):
        subframe = flow_module.recv_match(type='ENCAPSULATED_DATA', blocking=True)
        if subframe is None:
            continue
        else:
            frame = frame + subframe.data

    return np.array(frame[:2048], dtype=np.int8)

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
    try:
        flow = retrieveFlow()
        curr = retrieveFrame()
        tik.update()

    except:
        print("packet lost")
        continue

    fbFlow = cv2.calcOpticalFlowFarneback(prvs, curr, None, 0.5, 8, 15, 3, 5, 1.2, 0)

    if localcounter >= 20:
        tik.stop()
        fps = tik.fps()
        tik.reset()
        tik.start()
        localcounter = 0

        print(fps)
        smallshow = cv2.cvtColor(prvs, cv2.COLOR_GRAY2BGR)
        show = cv2.resize(smallshow, (684, 684), interpolation=cv2.INTER_LINEAR)
        index = 0
        for y in range(113, 8, -3):
            for x in range(9, 114, 3):
                show = cv2.line(show, (x*6, y*6), (x*6+int(fbFlow[y][x][0]*6), y*6+int(fbFlow[y][x][1]*6)), color=(10, 250, 10))
                show = cv2.line(show, (x*6, y*6), (x*6+flow[2*index]*6, y*6+flow[2*index+1]*6), color=(10, 10, 250))
                index += 1
        cv2.putText(show, str(int(fps)), (20, 20), cv2.FONT_HERSHEY_PLAIN, 0.8, (0, 150, 0))
        cv2.imshow('PX4FLOW', show)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        prvs = curr


cv2.destroyAllWindows()
flow_module.close()
