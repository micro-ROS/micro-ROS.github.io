---
permalink: /docs/home/
title: Overview
---

[micro-ROS](https://cordis.europa.eu/project/rcn/213167_en.html) puts ROS2 onto microcontrollers, making them first class participants of the ROS 2 environment.

## Tutorials

See [Tutorial List](/docs/tutorials/).

## Architecture

We're basically following the ROS2 architecture and make use of its middleware pluggability to use [DDS-XRCE](https://www.omg.org/spec/DDS-XRCE/), which is suitable for microcontrollers. Moreover, we use an RTOS (NuttX) instead of Linux. On microcontrollers many different RTOS's are used, so we also add an RTOS abstraction layer (AL), to make porting to other RTOS possible.

![](/img/micro-ROS_architecture.png)

For comaprisons to other approaches, see our [comparison](comparison) document.

## Concept Documentation

 - Predictable scheduling and execution:  [real-time_executor](/docs/real-time_executor/)
 - System modes and system hierarchy: [system_modes](/docs/system_modes/)
 - Embedded transform (tf) library: [embedded_tf](/docs/embedded_tf/)
 - Reference hardware: [https://github.com/microROS/hardware](https://github.com/microROS/hardware)
 - Build infrastructure for embedded development using docker: [https://github.com/microROS/docker](https://github.com/microROS/docker)
 - FIWARE interoperability: [FIROS2](/docs/FIROS2)

## Source Code Repositories

 - Middleware
  -  RMW adapter for Micro-XRCE-DDS [rmw-microxrcedds](https://github.com/microROS/rmw-microxrcedds)
  - Type Support [rosidl_typesupport_microxrcedds](https://github.com/microROS/rosidl_typesupport_microxrcedds)
  - "Agent" (bridge) [micro-ROS-Agent](https://github.com/microROS/micro-ROS-Agent)
 - RTOS
     - Our NuttX fork [https://github.com/microROS/NuttX](https://github.com/microROS/NuttX)
     - Example applications for NuttX [apps](https://github.com/microROS/apps)
 - Client library related
  - 'C' language client library (alpha!): [rclc](https://github.com/microROS/rclc)
  - Sample code for rclc and rclpp [micro-ROS-demos](https://github.com/microROS/micro-ROS-demos)
