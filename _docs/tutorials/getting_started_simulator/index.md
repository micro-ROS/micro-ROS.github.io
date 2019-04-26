---
title: Getting Started with simulator (WIP)
permalink: /docs/tutorials/getting_started_simulator/
author: Iñigo Muguruza Goenaga, Juan Muñoz Flores
---

Table of contents

*   [Running micro-ROS example in the simulator](#running-micro-ros-example)
*   [Running Micro XRCE-DDS example in the simulator](#running-micro-xrce-dds-example)

## Introduction

The aim of this section is to be able of running micro-ROS architecture examples using Docker files. The main difference at this examples is that you are going to use [QEMU simulator](https://www.qemu.org/).

In this simulator, we are going to emulate the architectures of the MCUs we are using at the project, which are Cortex-M3 and Cortex-M4. At the time of writing this page, as the Cortex-M4 is not supported, we are using a STMF103C8 part number configuration, with a bigger RAM and Flash size, in order to simulate the memory capabilities the used embedded board contains.



## Running micro-ROS example

WIP, is not running properly yet! More info [here](https://github.com/microROS/docker/tree/docker_sim/Simulation/micro-ROS).


## Running Micro XRCE-DDS example

This is a Docker image that sets the proper configuration and dependencies to run a Micro XRCE-DDS NuttX Client application in an emulated Cortex M3 MCU using Quemu.

This example uses NuttX RTOS, micro-XRCE-DDS client example code and micro-XRCE-DDS Agent.


### Build

Build your own the image using the docker file:

    ```bash
    docker build -t microxrcedds_sim .
    ```


### Run

_Note: the example explained here works with the [commit a495c65](https://github.com/eProsima/Micro-XRCE-DDS-Agent/commit/a495c65faa964ddc068ac6e1249f17f5c9f92787) of Micro XRCE-DDS Agent and NuttX "nuttx-sim-dev" branch [commit 54b0ca9](https://github.com/microROS/NuttX/commit/54b0ca954ae2a734ebef6d1dec589813bfd026a7)._

#### Client 1: Micro XRCE-DDS publisher

Open a new terminal and access the container with privileges so that we can run the binary in simulated board at Quemu:

```bash
docker run -it -v /dev/bus/usb:/dev/bus/usb --privileged microxrcedds_sim /bin/bash
```

 Execute the simulator running the NuttX binary that contains the RTOS and the client application:

```bash
##### inside the docker container
cd nuttx
qemu-system-arm -M stm32-f103c8 -serial pty -serial tcp::7777,server -kernel nuttx.bin
```

You should see something like this:

```bash
root@80b9715bedfe:~/nuttx# qemu-system-arm -M stm32-f103c8 -serial pty -serial tcp::7777,server -kernel nuttx.bin

(process:118): GLib-WARNING **: 17:19:54.688: ../../../../glib/gmem.c:489: custom memory allocation vtable not supported
char device redirected to /dev/pts/7 (label serial0)
QEMU waiting for connection on: tcp:0.0.0.0:7777,server
VNC server running on `127.0.0.1:5900'
LED Off

```


Qemu creates one serial port, labeled as "/dev/pts/x".This serial port is the auxiliary port, to be used to attach the Agent. The TCP server which have 0.0.0.0 IP and is attached to the port 7777 handles the console.
(If you want to use another Qemu simulation in the same machine you should give another port number I.E. 8888).
To access to the console, just use an utility like ``Netcat``. For this app, the command to execute should be:
```bash
netcat 0.0.0.0 7777
```

At this point, the simulator is running properly. In a new terminal, obtain the container ID typing `docker ps`. Find *microxrcedds_sim* and copy the *CONTAINER ID*. Type the next command to run an auxiliary  console: `docker exec -it <container_id> /bin/bash`.


The micro-ROS Agent is going to connect to the serial the simulator has opened to have communications among them. Now follow the next steps to execute the agent:

+ Access to the folder where the Agent is compiled, using `~/micro-XRCE-DDS-agent/build`.
+ Execute `./MicroXRCEAgent serial /dev/pts/<tty_number>` command, where <tty_number> is the second tty printed when the NuttX binary is executed. For example:

```
(process:118): GLib-WARNING **: 17:19:54.688: ../../../../glib/gmem.c:489: custom memory allocation vtable not supported
char device redirected to /dev/pts/7 (label serial0)
QEMU waiting for connection on: tcp:0.0.0.0:7777,server
VNC server running on `127.0.0.1:5900'
LED Off

```

In this case `/dev/pts/7` is the serial where the client and the agent will talk, so the agent's serial needs to be attached there. You should see the next once you launch the Agent:

```
root@2bdee009f1b1:~/micro-XRCE-DDS-agent/build# ././MicroXRCEAgent serial /dev/pts/7
Serial agent initialization... OK
Enter 'q' for exit
```

Now we need another execution of the docker attached to the NuttX Shell serial session. This session will be used to launch the client inside NuttX.

+ Execute again `docker exec -it <container_id> /bin/bash` in a new terminal.
+ Open a minicom session attached to the first serial session, following the previous example, `docker exec -it <container_id> /bin/bash`.
+ Open a `netcat tcp listener` to opent the terminal attached to the serial created by the emulator, in this case, `netcat 0.0.0.0 7777`
+ Hit enter a couple of times, you should see NuttX NSH shell prompt, type `?` to see if you have the client binary:

```
nsh> ?
help usage:  help [-v] [<cmd>]

  [           dd          help        mh          sleep       xd          
  ?           echo        hexdump     mw          test        
  cat         exec        kill        pwd         true        
  cd          exit        ls          set         unset       
  cp          false       mb          sh          usleep      

Builtin Apps:
  client
```
Chek the available serial ports:

```
nsh> ls /dev
/dev:
 console                                                                        
 null                                                                           
 ttyS0                                                                          
 ttyS1
 ```
 And launch the client, attaching it to the second serial where the Agent is listening, using `client --serial /dev/ttyS1` command. The output should be the next:

 ```
 nsh> client --serial /dev/ttyS1                                                 
 Serial mode => dev: /dev/ttyS1                                                  
 Running Shapes Demo Client...
 ```

#### Client 2: Micro XRCE-DDS subscriber

Repeat the steps described at the publisher. You need to change the port of the Qemu TCP server, in the next command you can see an example:
`qemu-system-arm -M stm32-f103c8 -serial pty -serial tcp::8888,server -kernel nuttx.bin`

#### Creating the publisher and subscriber

_Note: the instructions have been taken from eProsima [documentation](https://micro-xrce-dds.readthedocs.io/en/latest/shapes_demo.html)_

Now that we have two Micro XRCE-DDS clients running, with one terminal attached to the NuttX shell and the Agent connected to the second serial, we are able to set the function of each one.

##### Publisher

Take one of the clients NuttX Shell session and type the next command: `create_session`. The output should be the next:

```
=>>  34: (key: AA BB CC DD | n: 0 |  0) [CREATE CLIENT | req: 0x0001 | obj: 0xF
=>> << [34]: 80 00 00 00 00 01 1A 00 00 01 FF FE 58 52 43 45 01 00 01 0F 1D 05 >
=>>  34: (key: AA BB CC DD | n: 0 |  0) [CREATE CLIENT | req: 0x0001 | obj: 0xF
=>> << [34]: 80 00 00 00 00 01 1A 00 00 01 FF FE 58 52 43 45 01 00 01 0F 1D 05 >
<<=  33: (key: AA BB CC DD | n: 0 |  0) [STATUS AGENT | req: 0x0001 | obj: 0xFF
<<= << [33]: 81 00 00 00 04 01 19 00 00 01 FF FE 00 00 58 52 43 45 01 00 01 0F >
Status: OK      
```

Now, type the command `create_participant 1`.The output should be the next:

```
<<=  33: (key: AA BB CC DD | n: 0 |  0) [STATUS AGENT | req: 0x0001 | obj: 0xFF
<<= << [33]: 81 00 00 00 04 01 19 00 00 01 FF FE 00 00 58 52 43 45 01 00 01 0F >
=>>  48: (key: AA BB CC DD | r:80 |  0) [CREATE |   | req: 0x000A | obj: 0x0011
=>> << [48]: 81 80 00 00 01 01 28 00 00 0A 00 11 01 01 00 00 19 00 00 00 64 65 >
=>>  12: (key: AA BB CC DD | n:80 |  0) [HEARTBEAT | first: 0 | last: 0] t: 988
=>> << [12]: 81 00 80 00 0B 01 04 00 00 00 00 00 >>                             
<<=  14: (key: AA BB CC DD | r:80 |  0) [STATUS | req: 0x000A | obj: 0x0011 | O
<<= << [14]: 81 80 00 00 05 01 06 00 00 0A 00 11 00 00 >>                       
Status: OK                                                                      
=>>  12: (key: AA BB CC DD | n:80 |  0) [ACKNACK | seq_num: 1 | bitmap: 0000000
=>> << [12]: 81 00 80 00 0A 01 04 00 01 00 00 00 >>                             
=>>  12: (key: AA BB CC DD | n:80 |  0) [HEARTBEAT | first: 0 | last: 0] t: 990
=>> << [12]: 81 00 80 00 0B 01 04 00 00 00 00 00 >>                             
<<=  12: (key: AA BB CC DD | n:80 |  0) [ACKNACK | seq_num: 1 | bitmap: 0000000
<<= << [12]: 81 00 80 00 0A 01 04 00 01 00 00 00 >>                             
<<=  12: (key: AA BB CC DD | n:80 |  0) [ACKNACK | seq_num: 1 | bitmap: 0000000
<<= << [12]: 81 00 80 00 0A 01 04 00 01 00 00 00 >>                             
<<=  12: (key: AA BB CC DD | n:80 |  0) [ACKNACK | seq_num: 1 | bitmap: 0000000
<<= << [12]: 81 00 80 00 0A 01 04 00 01 00 00 00 >>
```

Now, create a topic `create_topic 1 1`.

Now, create publisher `create_publisher 1 1`, and a data writer, `create_datawriter 1 1`.

##### Subscriber

Once we have the publisher working , we need to attach a subscriber to it, using the Micro XRCE-DDS Agent.

First run in this client, the NuttX application of the client:
```
nsh> client --serial /dev/ttyS1
Serial mode => dev: /dev/ttyS1
Running Shapes Demo Client...                                                   
```

Now that the client is running, use the next commands to attach to the client that is going to publish data:

+ `create_session`
+ `create_participant 1`

At this point, you should see in the terminal of both Agents that they have matched:

```
RTPS Participant matched 1.f.0.2.cb.0.0.0.0.0.0.0|0.0.1.c1
```

+ `create_subscriber 1 1`
+ `create_datareader 1 1`

And finally, request data to the publisher:

+ `request_data 1 128 5`

##### Publish data

Now the subscriber is listening and the Agents are matched, publish data using the next command: `write_data 1 128 200 200 40 BLUE`

In the subscriber terminal, you should see how it receives the data:

```
Receiving... [SHAPE_TYPE | color: BLUE | x: 200 | y: 200 | size: 40]            
=>>  12: (key: AA BB CC DD | n:80 |  0) [ACKNACK | seq_num: 5 | bitmap: 0000000
=>> << [12]: 81 00 80 00 0A 01 04 00 05 00 00 00 >>                             
<<=  12: (key: AA BB CC DD | n:80 |  0) [HEARTBEAT | first: 4 | last: 4] t: 551
<<= << [12]: 81 00 80 00 0B 01 04 00 04 00 04 00 >>                             
=>>  12: (key: AA BB CC DD | n:80 |  0) [ACKNACK | seq_num: 5 | bitmap: 0000000
=>> << [12]: 81 00 80 00 0A 01 04 00 05 00 00 00 >>
```


_Needs rework to explain better how to open new terminals_


