---
title: Comparison to related approaches
permalink: /docs/overview/comparison/
---


Micro-ROS is a project that aims to bring ROS2 to the micro-controllers world. Here, we will perform an analysis of the alternative options, and eventually show a comparison table.

## ROSSerial

ROSSerial is a protocol for wrapping standard ROS serialized messages and multiplexing multiple topics and services over a character device such as a serial port or network socket. In addition to a protocol definition, there are three types of packages found in this suite:

- Client Libraries: Client libraries allow users to easily get ROS nodes up and running on various systems. These clients are ports of the general ANSI C++ rosserial_client library. 

- ROS-side Interfaces: Devices running rosserial code require a node on the host machine to bridge the connection from the serial protocol to the more general ROS network. This package aims to accomplish this bridge task.

- Examples and Use Cases. 

It is worth saying that this option cannot be fully compared with micro-ROS because this approach is meant to work with ROS1, instead of micro-ROS which is focused on ROS2.

Reference: [ROSserial Wiki](http://wiki.ros.org/rosserial)

## RIOT-ROS2

RIOT-ROS2 is a modification of the main ROS2 stack, to make it able to run on microcontrollers thanks to the RIOT Operating System.

ROS2 is composed of several layers. Some have been modified to be able to run on the microcontroller, this is a list of the available layers for RIOS-ROS2 project:
- ROS Client Library bindings: RCLC
- ROS Client Library: RCL
- ROS MiddleWare: rmw_ndn
- ROS IDL Generators: generator_c
- ROS IDL Type Support: CBOR
- ROS IDL Interfaces:
    - common_interfaces
    - rcl_interfaces

As a final data, it looks like the development is frozen. Due to, the last commit  was on [July 2018](https://github.com/astralien3000/riot-ros2/commits/master).

Reference:[RIOT-ROS2](https://github.com/astralien3000/riot-ros2/wiki)

## Comparation table

|  | rosserial | RIOT-ROS2 | micro-ROS |
|-------|-----------|-----------|-----------|
| OS | bare-metal | RIOT | NuttX,freeRTOS and Zephyr |
| Communications architecture | Bridged | N/A| Bridged |
| Message format | ROS1 | N/A |CDR (from DDS) |
| Communication links | UART | UART | UART, SPI, IP (UDP), 6LowPAN, ... |
| Communication protocol | Custom | NDN | XRCE-DDS (or any rmw implementation) |
| Code Base | Independent implementation | Standard ROS 2 stack up to RCL | Standard ROS 2 stack up to RCL (RCLCPP coming) |
| Node API | Custom rosserial API | RCL,RCLC | RCL (soon RCLCPP) |
| Callback execution | Sequential, in order of messages | N/A | Choice of ROS 2 executors or MCU optimized executors |
| Timers | Not included | Not included | Normal ROS 2 timers |
| Time sync to host | Custom | N/A | NTP/PTP |
| Lifecycle | Not supported | Partial | Partial, full coming |
