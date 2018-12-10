---
layout: default
---

[micro-ROS](https://cordis.europa.eu/project/rcn/213167_en.html) puts ROS2 onto microcontrollers, making them first class participants of the ROS 2 environment.

From the level of ROS onwards, we strive to re-use as much as possible from ROS 2, and be as compatible with it as possible. In some areas, we will probably do custom implementations optimized for resource use. This will definitely include TF, and maybe other areas such as scheduling. These optimized implementations may also be interesting for use with "normal" ROS2.

We'll update this README as we proceed. For now, you can check out our work on:

 - RMW implementation using Micro XRCE-DDS middleware: [rmw-microxrcedds](https://github.com/microROS/rmw-microxrcedds)
 - Type support for Micro XRCE-DDS: [rosidl_typesupport_microxrcedds](https://github.com/microROS/rosidl_typesupport_microxrcedds)
 - Micro-ROS-Agent package implementation: [micro-ROS-Agent](https://github.com/microROS/micro-ROS-Agent)
 - ROS Client Library for the C language: [rclc](https://github.com/microROS/rclc)
 - Sample code using rclc and rclcpp implementations: [micro-ROS-demos](https://github.com/microROS/micro-ROS-demos)
 - Library to support the implementation of language-specific ROS Client Libraries: [rcl](https://github.com/microROS/rcl)
 - Real-Time Operating System: [https://github.com/microROS/NuttX](https://github.com/microROS/NuttX)
 - Micro-ROS in RTOS example applications: [apps](https://github.com/microROS/apps)
 - System modes and system hierarchy: [micro-system-modes](https://github.com/microROS/system_modes)
 - Reference hardware: [https://github.com/microROS/hardware](https://github.com/microROS/hardware)
 - Build infrastructure for embedded development using docker: [https://github.com/microROS/docker](https://github.com/microROS/docker)

### Interoperability

In the road-map of the project, interoperability tasks are also considered. Apart from ROS (1) and ROS 2, [H-ROS](https://acutronicrobotics.com/modularity/H-ROS/) interoperability is going to be also granted, thanks to HRIM.  [HRIM](https://acutronicrobotics.com/modularity/hrim/) is an information model for robots that facilitates interoperability among modules from different vendors of robot hardware. 

### Architecture
The micro-ROS architecture is a work in progress. It's modular and built with the following ingredients:

 - A Real-Time Operating System (RTOS). This includes at least NuttX, and possibly others.
 - An embedded communications middleware, at least [Micro XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS)
 - The [ROS client library](http://github.com/microROS/rcl)

A first approach (yet not final) of the architecture is represented below:

![](assets/img/micro-ROS_architecture.png)

Got questions, [open an issue](https://github.com/microROS/micro-ROS/issues/new) and we'll get back to you as soon as possible.
