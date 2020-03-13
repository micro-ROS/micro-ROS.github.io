---
title: Features Status
permalink: /docs/overview/features/
---

Status of micro-ROS features. The features list has been compiled from [https://index.ros.org/doc/ros2/Features/](https://index.ros.org/doc/ros2/Features/) and [https://index.ros.org/doc/ros2/Roadmap/](https://index.ros.org/doc/ros2/Roadmap/).

All *status in italics* point to on-going works or open issues. If you could make any contribution, please open a pull request at a relevant [micro-ROS repository](https://github.com/micro-ROS/) or contact us via [Slack](https://micro-ros.slack.com/).

Feature | Status
-- | --
Discovery, transport and serialization over DDS | Use of resource-optimized middleware standard [DDS for Extremely Resource Constrained Environments (DDS-XRCE)](https://www.omg.org/spec/DDS-XRCE/), implemented by [Micro XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS) and compatible with standard DDS via an XRCE Agent on connected stronger microprocessor.
Support for multiple DDS implementations, chosen at runtime | Support is possible in principle, but at compile-time only. So far there is only one DDS-XRCE implementation available, namely [Micro-DDS-XRCE](https://github.com/eProsima/Micro-XRCE-DDS).
Common core client library that is wrapped by language-specific libraries | micro-ROS uses the core library [rcl](https://github.com/ros2/rcl/) from ROS 2 mainly as-is. The [rclc](https://github.com/micro-ROS/rclc) package provides convenience functions and an executor for use of rcl+rclc as an API for the C programming language. The standard rclcpp may be used on MCUs with sufficient RAM.
Publish/subscribe over topics | Concept available as known from ROS 2. Convenience functions for creation of publishers and subscriptions in C provided in [rclc](https://github.com/micro-ROS/rclc). *However, currently, micro-ROS supports fixed-size message types only.*
Clients and services | Concept available as known from ROS 2. *Convenience functions not yet provided and current implementation supports fixed-size message types only.*
ROS 1 - ROS 2 communication bridge | Not applicable, but the standard ROS 1 - ROS 2 bridge can be used via agent on a stronger microprocessor to communicate with micro-ROS nodes.
Quality of service settings for handling non-ideal networks | Reliable and best-effort semantics available. Can be set at compile time. *Still under development.*
Inter- and intra-process communication using the same API | No shared-memory interprocess communication on the MCU available, but all communication is performed via the agent running on a connected microprocessor. *Efficient shared-memory communication on the MCU is considered as an important feature for future releases.*
Composition of node components at compile-, link- or dlopen-time | Composition at compile-time only. Composition at runtime would depend highly on the RTOS.
Support for nodes with managed lifecycles | Very basic support provided by [rcl_lifecycle](https://github.com/ros2/rcl/tree/master/rcl_lifecycle/) out of the box for micro-ROS. *Further convenience functions are planned.*
DDS-Security support | DDS security is supported at agent. *Security mechanisms in Micro-XRCE-DDS are planned for future releases.*
Command-line introspection tools using an extensible framework | From a remote microprocessor all standard ROS 2 tools can be used to introspect the micro-ROS nodes on an MCU. Micro-ROS nodes appear as ROS 2 nodes (by the agent). Note, however, that the node graph API is currently *not* available on the MCU.
Launch system for coordinating multiple nodes | No launch system for the micro-ROS nodes on an MCU available. Such system would depend highly on the RTOS. The system-modes concept developed with micro-ROS allows runtime configuration/orchestration of ROS 2 and micro-ROS nodes together.
Namespace support for nodes and topics | Available just as in ROS 2.
Static remapping of ROS names | *Should be available if passed as argument via standard rcl API - to be checked.*
Demos of an all-ROS 2 mobile robot | Demos of several ROS 2 + micro-ROS robots available. See [https://micro-ros.github.io/docs/tutorials/demos/](https://micro-ros.github.io/docs/tutorials/demos/).
Support for real-time code | The [rclc Executor](https://github.com/micro-ROS/rclc/tree/master/rclc) provides mechanisms for implementing real-time-critical applications with micro-ROS.
Support for "bare-metal" microcontrollers | Bringing ROS 2 onto MCUs is all that micro-ROS is about. A crucial difference to this requirement from the early design phase of ROS 2 is that micro-ROS assumes an RTOS (e.g., [FreeRTOS](https://www.freertos.org/), [Zephyr](https://www.zephyrproject.org/), or [NuttX](http://nuttx.apache.org/)).
IDL | Same message IDL as with ROS 2, but use of resource-optimized CDR serialization implementation named [Micro-CDR](https://github.com/eProsima/Micro-CDR).
Build system | Build systems of NuttX and FreeRTOS integrated with colcon. *Integration of Zephyr's build system with colcon is in progress.* The build system is likely the most fragile part of micro-ROS w.r.t. the long-term maintenance, due to the many dependencies.
Continuous Integration | Currently, the CI for micro-ROS is distributed to GitHub and Gitlab. *Until the end of 2020, all CI should be moved migrated completely to the new CI actions of GitHub.* Please note that those packages that are released for standard ROS 2 are also built and tested on [build.ros2.org](http://build.ros2.org/).
Documentation | High-level documentation at [micro-ros.github.io](https://micro-ros.github.io/). For detailled information please consult the README.md files in the relevant micro-ROS repositories at [github.com/micro-ROS/](https://github.com/micro-ROS/).
Logging | *Could be available as part of the standard logging mechanism in principle but not supported by Micro-XRCE-DDS due to dynamic message size. To be checked ...*
Time-related: Support of rate and sleep with system clock | rcl timers use POSIX API. Tested successfully on NuttX, but resolution is very low. Higher resolution could be achieved with hardware timers - which highly depends on the MCU and possibly the RTOS. *This feature requires further investigation.*
Time-related: Support for simulation time | *Might be supported out of the box, but needs to be checked.* We consider HIL setups with simulation time to be corner cases.
