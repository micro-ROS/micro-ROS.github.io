---
title: yArchitecture
permalink: /docs/overview/
redirect_from: /docs/
---

*micro-ROS puts ROS 2 onto microcontrollers, making them first class participants of the ROS 2 environment.*

We're basically following the [ROS 2 architecture](https://index.ros.org/doc/ros2/) and make use of its middleware pluggability to use [DDS-XRCE](https://www.omg.org/spec/DDS-XRCE/), which is suitable for microcontrollers. Moreover, we use an RTOS (NuttX) instead of Linux. We also add an RTOS abstraction layer, to make porting to other RTOS possible.

![](/img/micro-ROS_architecture.png)

We have also prepared a [comparison](/docs/overview/comparison) to other approaches, to see the important differences quickly.

## Major Source Code Repositories

* Applications:
  * Kobuki demo: [embedded robot driver](https://github.com/micro-ROS/apps/tree/kobuki_rcl_port/examples/kobuki) and [remote ROS 2 software](https://github.com/micro-ROS/micro-ROS_kobuki_demo)
  * [Temperature demo](https://github.com/micro-ROS/micro-ROS_temperature_publisher_demo)
* Client library:
  * Extensions to rcl, i.e. the ROS 2 C API: [rcl_executor](https://github.com/micro-ROS/rcl_executor), ...
  * Extensions for rclcpp: [system_modes](https://github.com/micro-ROS/system_modes/), [TF improvements](https://github.com/micro-ROS/geometry2), ...
* Middleware:
  * eProsima's open-source implementation of DDS-XRCE: [Micro-XRCE-DDS](Micro-XRCE-DDS)
  * RMW adapter for Micro-XRCE-DDS: [rmw-microxrcedds](https://github.com/micro-ROS/rmw-microxrcedds)
  * Type support for Micro-XRCE-DDS: [rosidl_typesupport_microxrcedds](https://github.com/micro-ROS/rosidl_typesupport_microxrcedds)
  * Agent (bridge) to ROS 2: [micro-ROS-Agent](https://github.com/micro-ROS/micro-ROS-Agent)
* RTOS:
  * Our [NuttX fork](https://github.com/micro-ROS/NuttX), but most additions were contributed back.
  * Example applications for NuttX directly are in [apps](https://github.com/micro-ROS/apps)

Most repositories can be found in GitHub's micro-ROS organization at [github.com/micro-ROS/](https://github.com/micro-ROS/).
