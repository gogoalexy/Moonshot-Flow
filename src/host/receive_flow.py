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

    return np.array(frame[:128], dtype=np.int8)



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
        for y in range(256, 16, -32):
            for x in range(16, 256, 32):
                canvas = cv2.line(canvas, (x, y), (x+flow[2*index], y+flow[2*index+1]), color=(180, 250, 180))
                index += 1

        if localcounter >= 10:
            tik.stop()
            fps = tik.fps()
            tik.reset()
            tik.start()
            localcounter = 0

        print(fps)
        cv2.putText(canvas, str(int(fps)), (20, 20), cv2.FONT_HERSHEY_PLAIN, 0.8, (0, 150, 0))
        canvas = cv2.resize(canvas, (512, 512))
        cv2.imshow('PX4FLOW', canvas)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    except:
        print("packet lost")
        continue


cv2.destroyAllWindows()
flow_module.close()
