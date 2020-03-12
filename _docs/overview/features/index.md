---
title: Features Status
permalink: /docs/overview/features/
---

Status of micro-ROS features. The features list has been compiled from [https://index.ros.org/doc/ros2/Features/](https://index.ros.org/doc/ros2/Features/) and [https://index.ros.org/doc/ros2/Roadmap/](https://index.ros.org/doc/ros2/Roadmap/).

Feature | Status
-- | --
Discovery, transport and serialization over DDS | Implemented by XRCE-DDS.
Support for multiple DDS implementations, chosen at runtime | Possible in principle at compile-time. So far there is only one XRCE-DDS implementation, namely Micro-DDS-XRCE.
Common core client library that is wrapped by language-specific libraries | micro-ROS uses rcl from ROS 2, but provides convenience functions by rclc for use as a C API. rclcpp may be used on MCUs with sufficient RAM.
Publish/subscribe over topics | Implemented, fixed-size message types only.
Clients and services | Implemented, fixed-size message types only.
Set/retrieve parameters | Implemented. No convenience functions yet.
ROS 1 - ROS 2 communication bridge | N/A, possible via agent on microprocessor
Quality of service settings for handling non-ideal networks | Reliable and best-effort semantics available to be set at compile time. WIP
Inter- and intra-process communication using the same API | No shared-memory interprocess communication on the MCU but all communication is performed via the agent. We consider efficient shared-memory communication on the MCU as an important feature for the future.
Composition of node components at compile-, link- or dlopen-time | Composition at compile-time.
Support for nodes with managed lifecycles | tbd
DDS-Security support | DDS security is supported in agent. Some simple security in XRCE is planned.
Command-line introspection tools using an extensible framework | From a remote microprocessor all standard ROS 2 tools can be used to introspect the micro-ROS nodes on an MCU.micro-ROS nodes appear as ROS 2 nodes (by agent), the node graph API is not available on the MCU.
Launch system for coordinating multiple nodes | No launch system for the micro-ROS nodes on an MCU available (as this highly depends on the RTOS). System-modes concept developed in micro-ROS allows runtime configuration/orchestration of ROS 2 and micro-ROS nodes together.
Namespace support for nodes and topics | Just as in ROS 2.
Static remapping of ROS names | Should be available if passed as argument via standard rcl API - to be checked.
Demos of an all-ROS 2 mobile robot | Demo of an all ROS 2 + micro-ROS mobile robot (link to video)
Support for real-time code | micro-ROS offers rclc Executor which allows for deterministic, real-time guarantees.
Support for “bare-metal” microcontrollers | Bringing ROS 2 onto microcontrollers is all that micro-ROS is about - but with the difference that micro-ROS assumes an RTOS.

IDL | Same as with ROS 2, but own serialization named Micro CDR from XRCE-DDS standard
Building - build system | Build system for NuttX and FreeRTOS integrated with colcon. Integration with Zephyr WIPThis is the most fragile/risky part of micro-ROS w.r.t. the long-term maintenance. If the build system breaks with future updates of rcl, colcon, NuttX, FreeRTOS, Zephyr, then developers may get frustrated very soon and prefer small hand-written solutions over micro-ROS.
Building - CI | Currently distributed to GitHub and Gitlab.Until the end of 2020, all CI should be moved completely to GitHub, which offers CI actions since a few months.
Documentation | High-level documentation at micro-ros.github.io
Logging | Might be available as part of the standard logging mechanism in principle but not supported by Micro-XRCE-DDS due to dynamic message size. To be checked ...
Time-related: Support of rate and sleep with system clock | rcl timers use POSIX API. Tested successfully on NuttX, but resolution is very low. Higher resolution could be achieved with hardware timers - which highly depends on the MCU and possibly the RTOS. To be analyzed and implemented by at least one example. (Bosch)
Time-related: Support for simulation time | Might be supported out of the box. To be checked ... We consider HIL setups with simulation time to be corner cases.

