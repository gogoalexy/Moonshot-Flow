#!/bin/bash

git clone https://github.com/mavlink/mavlink.git --recursive
#PYTHONPATH=/home/ahyy/Drone/Moonshot-Flow/src/host/mavlink  in env/bin/bash
#python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=generated/include/mavlink/v2.0 message_definitions/v1.0/your_custom_dialect.xml
