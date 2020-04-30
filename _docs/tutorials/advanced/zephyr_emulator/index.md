---
title: Zephyr Emulator
permalink: /docs/tutorials/advanced/zephyr_emulator/
---

This tutorial aims at creating a new micro-ROS application on with **[Zephyr RTOS](https://www.zephyrproject.org/)** emulator (also known as [Native POSIX](https://docs.zephyrproject.org/latest/boards/posix/native_posix/doc/index.html)). 

To follow this tutorial, it is assumed that the user is already familiar with the **[First micro-ROS Application on an RTOS](https://micro-ros.github.io/docs/tutorials/core/first_application_rtos/)** tutorial. The target app in this tutorial is the same ping pong app.
Another requirement is that the user has a basic knowledge of micro-ROS and ROS 2.

<div>
<img  width="300" style="padding-right: 25px;" src="imgs/4.jpg">
</div>

## Required hardware

This tutorial requires no hardware beyond a Linux host computer.

## Building a Zephyr emulator application

Once the micro-ROS build system is ready, let's create a new Zephyr firmware for the host platform:

```bash
# Create firmware step
ros2 run micro_ros_setup create_firmware_ws.sh zephyr host
```


micro-ROS apps for Zephyr emulator are located at `firmware/zephyr_apps/apps`. In order to create a new application, create a new folder containing two files: the app code (inside a `src` folder) and the RMW configuration. You will also need some other Zephyr related files: a `CMakeLists.txt` to define the building process and a `prj.conf` where Zephyr is configured. There is a sample proyect [here](https://github.com/micro-ROS/zephyr_apps/tree/dashing/apps/host_ping_pong), for now, it is ok to copy them.

Make sure your app's `CMakeLists.txt` is compatible with `native_posix`:

```
...
set(COMPATIBLE_BOARDS native_posix)
....
```

Once the app folder is created, let's configure our new app with a UDP transport that looks for the agent on the port UDP/8888 at localhost:

```bash
# Configure step
ros2 run micro_ros_setup configure_firmware.sh host_ping_pong --transport udp --ip 127.0.0.1 --port 8888
```

When the configuring step ends, just build the firmware:

```bash
# Build step
ros2 run micro_ros_setup build_firmware.sh
```

Now you have a Zephyr + micro-ROS app ready to run on your own computer.

## Running the micro-ROS app

The micro-ROS app is ready to connect to a micro-ROS-Agent and start talking with the rest of the ROS 2 world.

First of all, create and build a micro-ROS agent:

```bash
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build micro-ROS-Agent packages, this may take a while.
colcon build
source install/local_setup.bash
```

Then run the agent:

```bash
# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

And run the Zephyr app in another command line (remember sourcing ROS 2 and micro-ROS installation): 

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source microros_ws/install/local_setup.bash

# Flash/run step
ros2 run micro_ros_setup flash_firmware.sh
```

And finally, let's check that everything is working in another command line. We are going to listen to ping topic to check whether the Ping Pong node is publishing its own pings:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Subscribe to micro-ROS ping topic
ros2 topic echo /microROS/ping
```

You should see the topic messages published by the Ping Pong node every 5 seconds:

```
user@user:~$ ros2 topic echo /microROS/ping
stamp:
  sec: 20
  nanosec: 867000000
frame_id: '1344887256_1085377743'
---
stamp:
  sec: 25
  nanosec: 942000000
frame_id: '730417256_1085377743'
---
```

On another command line, let's subscribe to the pong topic

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Subscribe to micro-ROS pong topic
ros2 topic echo /microROS/pong
```

At this point, we know that our app is publishing pings. Let's check if it also answers to someone else pings in a new command line:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Send a fake ping
ros2 topic pub --once /microROS/ping std_msgs/msg/Header '{frame_id: "fake_ping"}'
```

Now, we should see on the ping subscriber our fake ping along with the board pings:

```
user@user:~$ ros2 topic echo /microROS/ping
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
stamp:
  sec: 305
  nanosec: 973000000
frame_id: '451230256_1085377743'
---
stamp:
  sec: 310
  nanosec: 957000000
frame_id: '2084670932_1085377743'
---
```

And in the pong subscriber, we should see the board's answer to our fake ping:

```
pgarrido@pgarrido:~$ ros2 topic echo /microROS/pong
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
```

## Multiple Ping Pong nodes

One of the advantages of having an emulator is that you don't need to buy a bunch of hardware in order to test some multi-node micro-ROS apps. So, with the same micro-ROS agent of the last section, let's open four different command lines and run the following on each:

```bash
cd microros_ws

# This is an alternative way of executing the Zephyr emulator
./firmware/build/zephyr/zephyr.exe
```

As soon as all micro-ROS node are up and connected to the micro-ROS agent you will see them interacting:

```
pgarrido@pgarrido$ ./firmware/build/zephyr/zephyr.exe
*** Booting Zephyr OS build zephyr-v2.2.0-492-gc73cb85b4ae9  ***
UDP mode => ip: 127.0.0.1 - port: 8888
Ping send seq 1711620172_1742614911                         <---- This micro-ROS node sends a ping with ping ID "1711620172" and node ID "1742614911"
Pong for seq 1711620172_1742614911 (1)                      <---- The first mate pongs my ping 
Pong for seq 1711620172_1742614911 (2)                      <---- The second mate pongs my ping 
Pong for seq 1711620172_1742614911 (3)                      <---- The third mate pongs my ping 
Ping received with seq 1845948271_546591567. Answering.     <---- A ping is received from a mate identified as "546591567", let's pong it.
Ping received with seq 232977719_1681483056. Answering.     <---- A ping is received from a mate identified as "1681483056", let's pong it.
Ping received with seq 1134264528_1107823050. Answering.    <---- A ping is received from a mate identified as "1107823050", let's pong it.
Ping send seq 324239260_1742614911
Pong for seq 324239260_1742614911 (1)
Pong for seq 324239260_1742614911 (2)
Pong for seq 324239260_1742614911 (3)
Ping received with seq 1435780593_546591567. Answering.
Ping received with seq 2034268578_1681483056. Answering.
```

***TIP:** use the help flag to discover some Zephyr emulation features `./firmware/build/zephyr/zephyr.exe -h`*
