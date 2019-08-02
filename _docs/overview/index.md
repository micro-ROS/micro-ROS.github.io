---
title: Mission and Architecture
permalink: /docs/overview/
redirect_from: /docs/
---

*micro-ROS puts ROS 2 onto microcontrollers, making them first class participants of the ROS 2 environment.*

We're basically following the [ROS 2 architecture](https://index.ros.org/doc/ros2/) and make use of its middleware pluggability to use [DDS-XRCE](https://www.omg.org/spec/DDS-XRCE/), which is suitable for microcontrollers. Moreover, we use an RTOS (NuttX) instead of Linux. We also add an RTOS abstraction layer, to make porting to other RTOS possible.

![](/img/micro-ROS_architecture.png)

We have also prepared a [comparison](/docs/overview/comparison) to other approaches, to see the important differences quickly.

## Concept Documentation

 - Predictable scheduling and execution:  [real-time_executor](/docs/concepts/client_library/real-time_executor/)
 - System modes and system hierarchy: [system_modes](/docs/concepts/client_library/system_modes/)
 - Embedded transform (tf) library: [embedded_tf](/docs/concepts/client_library/embedded_tf/)
 - Reference hardware: [https://github.com/microROS/hardware](https://github.com/microROS/hardware)
 - Build infrastructure for embedded development using docker: [https://github.com/microROS/docker](https://github.com/microROS/docker)
 - FIWARE interoperability: [FIROS2](/docs/concepts/middleware/FIROS2)

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
