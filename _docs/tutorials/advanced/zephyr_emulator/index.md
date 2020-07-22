---
title: Zephyr Emulator
permalink: /docs/tutorials/advanced/zephyr_emulator/
---

## Target platform

In this tutorial, you'll learn the use of micro-ROS with a **[Zephyr RTOS](https://www.zephyrproject.org/)**
emulator (also known as [Native POSIX](https://docs.zephyrproject.org/latest/boards/posix/native_posix/doc/index.html))
by testing a Ping Pong application.

<div>
<img  width="300" style="padding-right: 25px;" src="imgs/4.jpg">
</div>

{% include first_application_common/build_system.md %}

```bash
# Create step
ros2 run micro_ros_setup create_firmware_ws.sh zephyr host
```

Once the command is executed, a folder named `firmware` must be present in your workspace.

This step is in charge, among other things, of creating a set of micro-ROS apps for the specific platform you are
addressing.
In the case of Zephyr, these are located at `firmware/zephyr_apps/apps`.
Each app is represented by a folder containing the following files:

* `src/main.c`: This file contains the logic of the application.
* `app-colcon.meta`: This file contains the micro-ROS app specific colcon configuration. Detailed info on how to
  configure the RMW via this file can be found
  [here](https://micro-ros.github.io/docs/tutorials/core/microxrcedds_rmw_configuration/).
* `CMakeLists.txt`: This is the CMake file containing the script to compile the application.
* `prj.conf`: This is a Zephyr specific app configuration file.

For the user to create its custom application, a folder `<my_app>` will need to be registered in this location,
containing the four files just described.

{% include first_application_common/config.md %}

In this tutorial, we will use a UDP transport that looks for the agent on the port UDP/8888 at localhost, and focus on
the out-of-the-box `host_ping_pong` application located at `firmware/zephyr_apps/apps/host_ping_pong`.
To execute this application with the chosen transport, run the configuration command above by specifying the `[APP]`
and `[OPTIONS]` parameters as below::

```bash
# Configure step
ros2 run micro_ros_setup configure_firmware.sh host_ping_pong --transport udp --ip 127.0.0.1 --port 8888
```

You can check the complete content of the `host_ping_pong` app
[here](https://github.com/micro-ROS/zephyr_apps/tree/dashing/apps/host_ping_pong).

{% include first_application_common/pingpong_logic.md %}

The contents of the Zephyr app specific files can be found here:
[main.c](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/host_ping_pong/src/main.c),
[app-colcon.meta](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/host_ping_pong/app-colcon.meta),
[CMakeLists.txt](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/host_ping_pong/CMakeLists.txt)
and [prj.conf](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/host_ping_pong/prj.conf).
A thorough review of these files is illustrative of how to create a micro-ROS app in this RTOS.

## Building the firmware

When the configuring step ends, just build the firmware:

```bash
# Build step
ros2 run micro_ros_setup build_firmware.sh
```

Now you have a Zephyr + micro-ROS app ready to run on your own computer.
Notice that in this case, the steps of flashing the firmware and running the micro-ROS app go together

## Flashing the firmware

Now you can run the flash step by means of the flashing command:

```bash
# Flash/run step
ros2 run micro_ros_setup flash_firmware.sh
```

Notice that this command also has the effect of running the micro-ROS node.

## Creating the micro-ROS agent

The micro-ROS app is now ready to be connected to a micro-ROS agent to start talking with the rest of the ROS 2
world.

To do that, let's open a new command line and create a micro-ROS agent
(remember sourcing the ROS 2 and micro-ROS installations):

```bash
# Download micro-ROS-Agent packages
source /opt/ros/dashing/setup.bash
source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
```

Now, let's build the agent packages and, when this is done, source the installation:

```bash
# Build step
colcon build
source install/local_setup.bash
```

## Running the micro-ROS app

At this point, you have both the client and the agent correctly installed in your host machine.

To start micro-ROS, you just need to run the agent:

```bash
# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## Testing the micro-ROS app

At this point, the micro-ROS app is built and flashed and the board is connected to a micro-ROS agent.
We now want to check that everything is working.

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

At this point, we know that our app is publishing pings.
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
user@user:~$ ros2 run micro_ros_setup flash_firmware.sh
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

One of the advantages of having an emulator is that you don't need to buy a bunch of hardware in order to test some
multi-node micro-ROS apps. So, with the same micro-ROS agent of the last section, let's open four different command
lines and run the following on each:

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
