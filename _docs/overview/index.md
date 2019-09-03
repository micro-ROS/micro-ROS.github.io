---
title: Architecture
permalink: /docs/overview/
redirect_from: /docs/
---

*micro-ROS puts ROS 2 onto microcontrollers, making them first class participants of the ROS 2 environment.*

We're basically following the [ROS 2 architecture](https://index.ros.org/doc/ros2/) and make use of its middleware pluggability to use [DDS-XRCE](https://www.omg.org/spec/DDS-XRCE/), which is suitable for microcontrollers. Moreover, we use an POSIX-based RTOS (NuttX) instead of Linux. We also add RTOS abstractions, to make porting to other RTOS easier.

![](/img/micro-ROS_architecture.png)

Dark blue components are developed specifically for micro-ROS. Light blue components are taken from the standard ROS 2 stack. We seek to contribute as much code back to the ROS 2 mainline codebase as possible.

We have also prepared a [comparison](/docs/overview/comparison) to other approaches, to see the important differences quickly.

## Source Code Repositories

Major repositories in order of the layers are:

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

### Compatibility with ROS 2 Releases

The following table summarizes the compatibility between micro-ROS and ROS2 releases.
This table contains links to installation documentation as well as `.repos` file for both *Client* and *Agent* side.
On the one hand, the *Client* size `.repos` includes all the necessary to build a micro-ROS application, along with some demo examples.
On the other hand, the *Agent* size `.repos` includes the micro-ROS-Agent bridge.

| ROS2 Release | Documentation | Client | Agent  |
|:-------------|:--------------|:-------|:-------|
| **Crystal**  | [![](https://img.shields.io/badge/read-the%20docs-blue)](https://github.com/micro-ROS/micro-ROS-doc/blob/crystal/Installation) | [![](https://img.shields.io/badge/uROS-repos-brightgreen)](https://github.com/micro-ROS/micro-ROS-doc/blob/crystal/Installation/repos/client_minimum.repos) | [![](https://img.shields.io/badge/uROS-repos-brightgreen)](https://github.com/micro-ROS/micro-ROS-doc/blob/crystal/Installation/repos/agent_minimum.repos) |
| **Dashing**  | [![](https://img.shields.io/badge/read-the%20docs-blue)](https://github.com/micro-ROS/micro-ROS-doc/blob/dashing/Installation) | [![](https://img.shields.io/badge/uROS-repos-brightgreen)](https://github.com/micro-ROS/micro-ROS-doc/blob/dashing/Installation/repos/client_minimum.repos) | [![](https://img.shields.io/badge/uROS-repos-brightgreen)](https://github.com/micro-ROS/micro-ROS-doc/blob/dashing/Installation/repos/agent_minimum.repos) |
