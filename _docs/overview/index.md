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
  * Broken URL: Temperature demo
* Client library:
  * Extensions to rcl, i.e. the ROS 2 C API: [rcl_executor](https://github.com/micro-ROS/rcl_executor), ...
  * Extensions for rclcpp: [system_modes](https://github.com/micro-ROS/system_modes/), [TF improvements](https://github.com/micro-ROS/geometry2), ...
* Middleware:
  * eProsima's open-source implementation of DDS-XRCE: [Micro-XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS)
  * RMW adapter for Micro-XRCE-DDS: [rmw-microxrcedds](https://github.com/micro-ROS/rmw-microxrcedds)
  * Type support for Micro-XRCE-DDS: [rosidl_typesupport_microxrcedds](https://github.com/micro-ROS/rosidl_typesupport_microxrcedds)
  * Agent (bridge) to ROS 2: [micro-ROS-Agent](https://github.com/micro-ROS/micro-ROS-Agent)
* RTOS:
  * Our [NuttX fork](https://github.com/micro-ROS/NuttX), but most additions were contributed back.
  * Example applications for NuttX directly are in [apps](https://github.com/micro-ROS/apps)

Most repositories can be found in GitHub's micro-ROS organization at [github.com/micro-ROS/](https://github.com/micro-ROS/).

### List of Repositories

| Name                            | Documentation | Release | CI | Issues |
|:--------------------------------|:--------------|:--------|:---|:-------|
| rmw-microxrcedds                | [![](https://img.shields.io/badge/read-the%20docs-blue)](https://github.com/micro-ROS/rmw-microxrcedds/blob/master/README.md) | [![](https://img.shields.io/badge/ROS-crystal-brightgreen)](https://github.com/micro-ROS/rmw-microxrcedds/tree/release-crystal-20190312) | | [![](https://img.shields.io/github/issues/micro-ROS/rmw-microxrcedds)](https://github.com/micro-ROS/rmw-microxrcedds/issues) |
| rosidl_typesupport_microxrcedds |  [![](https://img.shields.io/badge/read-the%20docs-blue)](https://github.com/micro-ROS/rosidl_typesupport_microxrcedds/blob/master/README.md) | [![](https://img.shields.io/badge/ROS-crystal-brightgreen)](https://github.com/micro-ROS/rosidl_typesupport_microxrcedds/blob/release-crystal-20190312/README.md) |   | [![](https://img.shields.io/github/issues/micro-ROS/rosidl_typesupport_microxrcedds)](https://github.com/micro-ROS/rosidl_typesupport_microxrcedds/issues) |
| micro-ROS-Agent                 |  [![](https://img.shields.io/badge/read-the%20docs-blue)](https://github.com/micro-ROS/micro-ROS-Agent/blob/master/README.md) | [![](https://img.shields.io/badge/ROS-crystal-brightgreen)](https://github.com/micro-ROS/micro-ROS-Agent/blob/release-crystal-20190312/README.md) |  [![](http://build.ros2.org/buildStatus/icon?job=Cbin_uB64__micro-xrce-dds-agent__ubuntu_bionic_amd64__binary)](https://github.com/micro-ROS/micro-ROS-doc/blob/crystal/Installation/repos/agent_minimum.repos) | [![](https://img.shields.io/github/issues/micro-ROS/micro-ROS-Agent)](https://github.com/micro-ROS/micro-ROS-Agent/issues) |
| Micro XRCE-DDS                  | [![](https://img.shields.io/badge/read-the%20docs-blue)](https://micro-xrce-dds.readthedocs.io/en/latest/) | [![](https://img.shields.io/badge/ROS-crystal-brightgreen)](https://github.com/eProsima/Micro-XRCE-DDS/tree/v1.0.3) [![](https://img.shields.io/badge/ROS-dashing-brightgreen)](https://github.com/eProsima/Micro-XRCE-DDS/tree/v1.1.0) |    | [![](https://img.shields.io/github/issues/eProsima/Micro-XRCE-DDS.svg)](https://github.com/eProsima/Micro-XRCE-DDS/issues) |
| system_modes                    | [![](https://img.shields.io/badge/read-the%20docs-blue)](https://github.com/micro-ROS/system_modes/blob/master/README.md) | [![](https://img.shields.io/badge/ROS-crystal-brightgreen)](https://github.com/micro-ROS/system_modes/releases) [![](https://img.shields.io/badge/ROS-dashing-brightgreen)](https://github.com/micro-ROS/system_modes/releases) [![](https://img.shields.io/badge/ROS-eloquent-brightgreen)](https://github.com/micro-ROS/system_modes/releases) | [![Build Status](http://build.ros2.org/job/Ddev__system_modes__ubuntu_bionic_amd64/badge/icon)](http://build.ros2.org/job/Ddev__system_modes__ubuntu_bionic_amd64/) | [![](https://img.shields.io/github/issues/micro-ROS/system_modes.svg)](https://github.com/micro-ROS/system_modes/issues) |

