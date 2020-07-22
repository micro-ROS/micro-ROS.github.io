---
title: First micro-ROS Application on Linux
permalink: /docs/tutorials/core/first_application_linux/
---

## Target platform

This tutorial teaches you how to try a first micro-ROS application on Linux, by testing a Ping Pong application.
In the follow-up tutorial [*First micro-ROS application on an RTOS*](/docs/tutorials/core/first_application_rtos/),
you'll learn how to build and bring this application on a microcontroller running the RTOS NuttX, FreeRTOS, or Zephyr.
Finally in the tutorial [*Zephyr Emulator*](/docs/tutorials/advanced/zephyr_emulator/) you'll learn how to test
a micro-ROS application on a Zephyr emulator.

{% include first_application_common/build_system.md %}

```bash
# Create firmware step
ros2 run micro_ros_setup create_firmware_ws.sh host
```

Once the command is executed, a folder named `firmware` must be present in your workspace.

This step is in charge, among other things, of creating a set of micro-ROS apps for Linux, that are located at
`src/uros/micro-ROS-demos/rcl`.
Each app is represented by a folder containing the following files:

* `app.c`: The former contains the logic of the application.
* `CMakeLists.txt`: This is the CMake file containing the script to compile the application.

For the user to create its custom application, a folder `<my_app>` will need to be registered in this location,
containing the two files just described.
Also, any such new application folder needs to be registered in
`src/uros/micro-ROS-demos/rcl/CMakeLists.txt` by adding the following line:

```
export_executable(<my_app>)
```

In this tutorial, we will focus on the out-of-the-box `ping_pong` application located at
`src/uros/micro-ROS-demos/rcl/ping_pong`.
You can check the complete content of this app
[here](https://github.com/micro-ROS/micro-ROS-demos/tree/dashing/rcl/ping_pong).

{% include first_application_common/pingpong_logic.md %}

The app logic of this demo is contained in
[`app.c`](https://github.com/micro-ROS/micro-ROS-demos/blob/dashing/rcl/ping_pong/main.c).
A thorough review of this file is illustrative of how to create a micro-ROS publisher or subscriber.

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

## Running the micro-ROS app

At this point, you have both the client and the agent correctly installed in your host machine.

To start micro-ROS, first run the agent:

```bash
# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

And then, in another command line, run the micro-ROS node (remember sourcing the ROS 2 and micro-ROS installations):

```bash
source /opt/ros/dashing/setup.bash
source install/local_setup.bash

# Run a micro-ROS agent
ros2 run micro_ros_demos_rcl ping_pong
```

## Testing the micro-ROS app

Now, we want to check that everything is working.

Open a new command line. We are going to listen to the `ping` topic
with ROS 2 to check whether the micro-ROS Ping Pong node is correctly publishing the expected pings:

```bash
source /opt/ros/dashing/setup.bash

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

At this point, we know that our Linux app is publishing pings.
Let's check if it also answers to someone else's pings. If this works, it'll publish a pong.

So, first of all, let's subscribe with ROS 2 to the `pong` topic
(notice that initially we don't expect to receive any pong, since none has been sent yet):

```bash
source /opt/ros/dashing/setup.bash

# Subscribe to micro-ROS pong topic
ros2 topic echo /microROS/pong
```

And now, let's publish a `fake_ping` with ROS 2 from yet another command line:

```bash
source /opt/ros/dashing/setup.bash

# Send a fake ping
ros2 topic pub --once /microROS/ping std_msgs/msg/Header '{frame_id: "fake_ping"}'
```

Now, we should see this `fake_ping` in the `ping` subscriber console (the second that we opened),
along with the micro-ROS pings:

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

Also, we expect that, because of having received the `fake_ping`, the micro-ROS node will answer with a `pong`:

```
user@user:~$ ros2 run micro_ros_demos_rcl ping_pong
Ping send seq 1706097268_1085377743
Ping send seq 181171802_1085377743
Ping send seq 1385567526_1085377743
Ping send seq 926583793_1085377743
Ping send seq 1831510138_1085377743
Ping received with seq fake_ping. Answering.
Ping send seq 1508705084_1085377743
Ping send seq 1702133625_1085377743
Ping send seq 176104820_1085377743
```

As a consequence, in the `pong` subscriber console (the third that we opened),
we should see the micro-ROS app answer to our `fake_ping`:

```
user@user:~$ ros2 topic echo /microROS/pong
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
```

## Multiple Ping Pong nodes

One of the advantages of having an Linux micro-ROS app is that you don't need to buy a bunch of hardware in order to
test some multi-node micro-ROS apps.
So, with the same micro-ROS agent of the last section, let's open four different command lines and run the following on
each:

```bash
cd microros_ws

source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

ros2 run micro_ros_demos_rcl ping_pong
```

As soon as all micro-ROS nodes are up and connected to the micro-ROS agent you will see them interacting:

```
user@user$ ros2 run micro_ros_demos_rcl my_brand_new_app
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
