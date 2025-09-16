---
title: Zephyr Emulator
permalink: /docs/tutorials/core/zephyr_emulator/
---

<img src="https://img.shields.io/badge/Tested_on-Humble-green" style="display:inline"/>

In this tutorial, you'll learn the use of micro-ROS with a **[Zephyr RTOS](https://www.zephyrproject.org/)**
emulator (also known as [Native POSIX](https://docs.zephyrproject.org/4.1.0/boards/native/native_posix/doc/index.html))
by testing a Ping Pong application.

<div>
<img  width="300" style="padding-right: 25px;" src="/img/posts/logo-zephyr.jpg">
</div>

{% include first_application_common/build_system.md %}

```bash
# Create step
ros2 run micro_ros_setup create_firmware_ws.sh zephyr host
```

{% include first_application_common/zephyr_common.md %}

{% include first_application_common/config.md %}

In this tutorial, we will use a UDP transport that looks for the agent on the port UDP/8888 at localhost, and focus on
the out-of-the-box `ping_pong` application located at `firmware/zephyr_apps/apps/ping_pong`.
To execute this application with the chosen transport, run the configuration command above by specifying the `[APP]`
and `[OPTIONS]` parameters as below:

```bash
# Configure step
ros2 run micro_ros_setup configure_firmware.sh ping_pong --transport udp --ip 127.0.0.1 --port 8888
```

You can check the complete content of the `ping_pong` app
[here](https://github.com/micro-ROS/zephyr_apps/tree/humble/apps/ping_pong).

{% include first_application_common/pingpong_logic.md %}

The contents of the Zephyr app specific files can be found here:
[main.c](https://github.com/micro-ROS/zephyr_apps/blob/humble/apps/ping_pong/src/main.c),
[app-colcon.meta](https://github.com/micro-ROS/zephyr_apps/blob/humble/apps/ping_pong/app-colcon.meta),
[CMakeLists.txt](https://github.com/micro-ROS/zephyr_apps/blob/humble/apps/ping_pong/CMakeLists.txt)
and [host-udp.conf](https://github.com/micro-ROS/zephyr_apps/blob/humble/apps/ping_pong/host-udp.conf).
A thorough review of these files is illustrative of how to create a micro-ROS app in this RTOS.

## Building the firmware

When the configuring step ends, just build the firmware:

```bash
# Build step
ros2 run micro_ros_setup build_firmware.sh
```

Now you have a Zephyr + micro-ROS app ready to run on your own computer.
Notice that in this case, the steps of flashing the firmware and running the micro-ROS app go together.

{% include first_application_common/agent_creation.md %}

## Running the micro-ROS app

At this point, you have both the client and the agent correctly installed in your host machine.

To give micro-ROS access to the ROS 2 dataspace, run the agent:

```bash
# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

## Flashing the firmware

Finally, in order to run the micro-ROS node inside of the Zephyr RTOS emulator,
open a new command shell and execute the flash step by means of the flashing command:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
source install/local_setup.bash

# Flash/run step
ros2 run micro_ros_setup flash_firmware.sh
```

{% include first_application_common/test_app_host.md %}

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
user@user:~$ ./firmware/build/zephyr/zephyr.exe
*** Booting Zephyr OS build zephyr-v2.2.0-492-gc73cb85b4ae9  ***
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
