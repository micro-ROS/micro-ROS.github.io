---
title: Micro-ROS configuration for NuttX
permalink: /docs/tutorials/advanced/nuttx/add_microros_config/
---

|  RTOS | ROS2 Version |
|:-----:|:------------:|
| NuttX |   Dashing   |

In this tutorial, we will see how to set a basic Micro-ROS configuration for NuttX over serial communication. Since this guide is only focused on setting the configuration, you should check the tutorial linked here before: [Adding Micro-ROS to a NuttX board configuration](https://micro-ros.github.io/docs/tutorials/advanced/nuttx/microros_nuttx_bsp/)

## Disclamer

This guide is not guarantee to work on every NuttX supported board, because each one has a different level of peripheral implementation and memory available.

## Required hardware

- Any NuttX supported board with at least these characteristics:
    - STM32 MCU.
    - RAM: 125 Kb.
    - Flash: 512 kb.
    - Serial communication peripheral.

- USB-TTY serial cable.

## Workspace set-up

We're going to create the Micro-ROS workspace, to do so, we will execute the following commands on a console:

```
source /opt/ros/$ROS_DISTRO/setup.bash

mkdir uros_ws && cd uros_ws

git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro-ros-build.git src/micro-ros-build

rosdep update && rosdep install --from-path src --ignore-src -y

colcon build

source install/local_setup.bash
```



## Set the base configuration

 In this guide, for example, we will give Micro-ROS support for the [Olimex-STM32-H407 board](https://www.olimex.com/Products/ARM/ST/STM32-H407/open-source-hardware).

This board is a simplified version of our supported Olimex-STM32-E407 board. 

On the previous console, execute the following commands to set the basic configuration:
```bash
ros2 run micro_ros_setup create_firmware_ws.sh nuttx olimex-stm32-h407
ros2 run micro_ros_setup configure_firmware.sh nsh
```

If everything goes fine, it should output the next message:
```bash
Copy files
Refreshing...
```

Now is set the basic NSH configuration. On the forward steps, we will set the required configuration to run Micro-ROS on this board.

## Micro-ROS configuration.

On the previous console execute the next commands:
```bash
cd firmware/NuttX
make menuconfig
```

This should show this menu:
![](images/nuttx_menuconfig.png)

Note: Remember that you should follow this previous tutorial before starting it. [Adding Micro-ROS to a NuttX board configuration](https://micro-ros.github.io/docs/tutorials/advanced/nuttx/microros_nuttx_bsp/)

On the menu, you need to set the next configuration.

- System Type > STM32 Peripheral Support > USART3
- RTOS Features > Clocks and Timers > Support CLOCK_MONOTONIC
- Device Drivers > Serial Driver Support > Serial TERMIOS support
- Library Routines > Standard Math Library
- Library Routines > sizeof(_Bool) is a 8-bits
- Library Routines > Build uClibc++ (must be installed) 
- Application Configuration > micro-ROS
- Application Configuration > micro-ROS > Transport > Serial Transport
- Application Configuration > Examples > microROS Publisher

Push two times the key "ESC" and set yes, when it asks you if you want to save it.

Now the configuration is properly set, is just left to compile it. To do so, continue on the same console and execute the commands:
```
cd ../..
ros2 run micro_ros_setup build_firmware.sh
```

If everything goes fine, it should return the next output:
```bash
CP: nuttx.hex
CP: nuttx.bin
```
