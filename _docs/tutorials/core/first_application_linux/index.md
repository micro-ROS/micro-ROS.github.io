---
title: First micro-ROS Application on Linux
permalink: /docs/tutorials/core/first_application_linux/
---

<img src="https://img.shields.io/badge/Written_for-Humble-green" style="display:inline"/> <img src="https://img.shields.io/badge/Tested_on-Rolling-green" style="display:inline"/> <img src="https://img.shields.io/badge/Tested_on-Iron-green" style="display:inline"/>

In this tutorial, you’ll learn the use of micro-ROS with Linux by testing a Ping Pong application.
In the follow-up tutorial [*First micro-ROS application on an RTOS*](/docs/tutorials/core/first_application_rtos/),
you'll learn how to build and bring this application on a microcontroller running the RTOS NuttX, FreeRTOS, or Zephyr.
Finally, in the tutorial [*Zephyr Emulator*](/docs/tutorials/core/zephyr_emulator/) you'll learn how to test
a micro-ROS application on a Zephyr emulator.

{% include first_application_common/build_system.md %}

```bash
# Create firmware step
ros2 run micro_ros_setup create_firmware_ws.sh host
```

Once the command is executed, a folder named `firmware` must be present in your workspace.

This step is in charge, among other things, of downloading a set of micro-ROS apps for Linux, that are located at
`src/uros/micro-ROS-demos/rclc`.
Each app is represented by a folder containing the following files:

* `main.c`: This file contains the logic of the application.
* `CMakeLists.txt`: This is the CMake file containing the script to compile the application.

For the user to create a custom application, a folder `<my_app>` will need to be registered in this location,
containing the two files just described.
Also, any such new application folder needs to be registered in
`src/uros/micro-ROS-demos/rclc/CMakeLists.txt` by adding the following line:

```
export_executable(<my_app>)
```

In this tutorial, we will focus on the out-of-the-box `ping_pong` application located at
`src/uros/micro-ROS-demos/rclc/ping_pong`.
You can check the complete content of this app
[here](https://github.com/micro-ROS/micro-ROS-demos/tree/humble/rclc/ping_pong).

{% include first_application_common/pingpong_logic.md %}

The contents of the host app specific files can be found here:
[main.c](https://github.com/micro-ROS/micro-ROS-demos/blob/humble/rclc/ping_pong/main.c) and
[CMakeLists.txt](https://github.com/micro-ROS/micro-ROS-demos/blob/humble/rclc/ping_pong/CMakeLists.txt).
A thorough review of these files is illustrative of how to create a micro-ROS app in this RTOS.

## Building the firmware

Once the app has been created, the build step is in order.
Notice that, with respect to the four-steps workflow delined above, we would expect a configuration step to happen
before building the app. However, given that we are compiling micro-ROS in the host machine rather than in a board,
the cross-compilation implemented by the configuration step is not required in this case.
We can therefore proceed to build the firmware and source the local installation:

```bash
# Build step
ros2 run micro_ros_setup build_firmware.sh
source install/local_setup.bash
```
{% include first_application_common/agent_creation.md %}

### Add micro-ROS environment to bashrc (optional)

You can add the ROS 2 and micro-ROS workspace setup files to your `.bashrc` so the files do not have to be sourced every time a new command line is opened.
```bash
echo source /opt/ros/$ROS_DISTRO/setup.bash >> ~/.bashrc
echo source ~/microros_ws/install/local_setup.bash >> ~/.bashrc
```

## Running the micro-ROS app

At this point, you have both the client and the agent correctly installed in your host machine.

To give micro-ROS access to the ROS 2 dataspace, run the agent:

```bash
# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

And then, in another command line, run the micro-ROS node (remember sourcing the ROS 2 and micro-ROS installations, and setting the RMW Micro XRCE-DDS implementation):

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

# Use RMW Micro XRCE-DDS implementation
export RMW_IMPLEMENTATION=rmw_microxrcedds

# Run a micro-ROS node
ros2 run micro_ros_demos_rclc ping_pong
```

{% include first_application_common/test_app_host.md %}

## Multiple Ping Pong nodes

One of the advantages of having a Linux micro-ROS app is that you don't need to buy a bunch of hardware in order to
test some multi-node micro-ROS apps.
So, with the same micro-ROS agent of the last section, let's open four different command lines and run the following on
each:

```bash
cd microros_ws

source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

export RMW_IMPLEMENTATION=rmw_microxrcedds

ros2 run micro_ros_demos_rclc ping_pong
```

As soon as all micro-ROS nodes are up and connected to the micro-ROS agent you will see them interacting:

```
user@user:~$ ros2 run micro_ros_demos_rclc ping_pong
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
