## PX4Flow Firmware

[![Build Status](https://travis-ci.org/PX4/Flow.svg?branch=master)](https://travis-ci.org/PX4/Flow)

[![Gitter](https://badges.gitter.im/Join%20Chat.svg)](https://gitter.im/PX4/Firmware?utm_source=badge&utm_medium=badge&utm_campaign=pr-badge&utm_content=badge)

PX4 FLOW is a smart camera processing optical flow directly on the camera module. It is optimized for processing and outputs images only for development purposes. Its main output is a UART or I2C stream of flow measurements at ~400 Hz.

Dev guide / toolchain installation:
https://docs.px4.io/master/en/sensor/px4flow.html

For help, run:

```bash
make help
```

To build, run:
```bash
  make archives # this needs to be done only once
  make
```

To flash via the PX4 bootloader (first run this command, then connect the board):
```bash
  make upload-usb
```

By default the px4flow-v1_default is uploaded; to upload a different version, run:

```bash
  make <target> upload-usb
```
Where `<target>` is one of the px4flow targets listed by `make help`.

### Attention
Here we use a non-standard mavlink protocol which supports signed data chunks transmission. Only a minor modification is made based on mavlink 1.0.12 and currently it is OK to use standard protocol. We still suggest you to build a costume pymavlink module from the source in the repository.

__Note:__ For archlinux, the first line in the file `Flow/src/lib/uavcan/libuavcan/dsdl_compiler/libuavcan_dsdlc` should be modified as `#!/usr/bin/env python2` and make sure the whole program use python2 not 3.
