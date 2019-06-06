---
title: Getting Started with an embedded board (WIP)
permalink: /docs/tutorials/basic/getting_started_embedded/
redirect_from: /docs/tutorials/
---

Table of contents

*   [Running micro-ROS example at an embedded board](#running-micro-ros-example)
*   [Running Micro XRCE-DDS example at an embedded board](#running-micro-xrce-dds-example)

## Introduction

The aim of this section is to be able of running micro-ROS architecture examples using Docker files at one of the supported embedded boards.

For that purpose, we are going to use a [Olimex STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware) board, that contains and Cortex-M4F microcontroller.


## Running micro-ROS example

At this example, we are going to flash the board with a binary that contains a NuttX App. This NuttX App is a micro-ROS client. This client will try to connect to the micro-ROS Agent running at your computer and exchange data using Micro XRCE-DDS.

For that purpose, firs install [this](https://github.com/microROS/docker/tree/master/Embedded/NuttX/precompiled/micro-XRCE-DDS/olimex-stm32-e407) container at your Linux machine.



Build the container and compile ready to work `micro XRCE-DDS` with a subscriber and a publisher for the `OLIMEX STM32-E407` board:
```bash
docker build -t microros_uxdf4 .
```
Access the container with privileges so that we can flash the board from within:
```bash
docker run -it -v /dev/bus/usb:/dev/bus/usb --privileged microros_uxdf4 /bin/bash
```
Flash the board:
```bash
# inside the docker container
cd nuttx
./scripts/flash.sh olimex-stm32-e407
```
if successful, you should see something like this:


```
cortex_m reset_config sysresetreq
Info : clock speed 2000 kHz
Info : JTAG tap: stm32f4x.cpu tap/device found: 0x4ba00477 (mfg: 0x23b (ARM Ltd.), part: 0xba00, ver: 0x4)
Info : JTAG tap: stm32f4x.bs tap/device found: 0x06413041 (mfg: 0x020 (STMicroelectronics), part: 0x6413, ver: 0x0)
Info : stm32f4x.cpu: hardware has 6 breakpoints, 4 watchpoints
adapter speed: 2000 kHz
Info : JTAG tap: stm32f4x.cpu tap/device found: 0x4ba00477 (mfg: 0x23b (ARM Ltd.), part: 0xba00, ver: 0x4)
Info : JTAG tap: stm32f4x.bs tap/device found: 0x06413041 (mfg: 0x020 (STMicroelectronics), part: 0x6413, ver: 0x0)
target halted due to debug-request, current mode: Thread
xPSR: 0x01000000 pc: 0x08000188 msp: 0x20006a70
auto erase enabled
Info : device id = 0x10076413
Info : flash size = 1024kbytes
wrote 131072 bytes from file nuttx.bin in 3.916489s (32.682 KiB/s)
```

_Need to complete the tutorial!_



## Running Micro XRCE-DDS example

In this case, instead of running the complete micro-ROS stuck, we will run a DDS level example. For that purpose, the embedded board will execute a Micro XRCE-DDS example that will communicate with an Agent that is running in your Linux machine.

First of all, downlod the [Docker machine](https://github.com/microROS/docker/blob/master/Embedded/NuttX/precompiled/micro-ROS/olimex_stm32-e407/Dockerfile) containing all the tools. Along with all the dependencies, this image contains micro-ROS publish/subscribe pre-built examples ready to flash and run on the board.

## Build

There are two options to get the image.

1. Build your own the image using the docker file:

    ```bash
    docker build -t microros_stm32f4 .
    ```

1. Pull the image from DockerHub.
    ```bash
    docker pull microros/stm32-e407
    ```

## Run

Access the container with privileges so that we can flash the board from within:

```bash
docker run -it -v /dev/bus/usb:/dev/bus/usb --privileged microros/stm32-e407 /bin/bash
```

 Flash the board:

```bash
# inside the docker container
cd nuttx
openocd -f interface/ftdi/olimex-arm-usb-tiny-h.cfg -f target/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase nuttx.bin 0x08000000"
```

if successful, you should see something like this:

```bash
.
.
.
Info : JTAG tap: stm32f4x.bs tap/device found: 0x06413041 (mfg: 0x020, part: 0x6413, ver: 0x0)
Info : stm32f4x.cpu: hardware has 6 breakpoints, 4 watchpoints
Info : JTAG tap: stm32f4x.cpu tap/device found: 0x4ba00477 (mfg: 0x23b, part: 0xba00, ver: 0x4)
Info : JTAG tap: stm32f4x.bs tap/device found: 0x06413041 (mfg: 0x020, part: 0x6413, ver: 0x0)
target state: halted
target halted due to debug-request, current mode: Thread
xPSR: 0x01000000 pc: 0x080004b4 msp: 0x20000de4
auto erase enabled
Info : device id = 0x10076413
Info : flash size = 1024kbytes
wrote 65536 bytes from file nuttx.bin in 2.157869s (29.659 KiB/s)

```

At this point, you should have your board flashed with NuttX image containing an `nsh` application and a publisher and a subscriber using Micro XRCE-DDS.

_To be completed!_
