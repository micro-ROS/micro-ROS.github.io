---
title: ROS 2 Feature Comparison
permalink: /docs/overview/ROS_2_feature_comparison/
---

Comparison of micro-ROS features with ROS 2 features. The following list has been compiled from [https://index.ros.org/doc/ros2/Features/](https://index.ros.org/doc/ros2/Features/) and [https://index.ros.org/doc/ros2/Roadmap/](https://index.ros.org/doc/ros2/Roadmap/).

<style>
  .status_flag {
    font-size: 150%;
    font-weight: bold;
  }
</style>

ROS 2 Feature | | Availability in micro-ROS
-- | -- | --
Transport and serialization over DDS | <span class="status_flag">&#10003;</span> | Provided by [Micro XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS) and compatible with standard DDS via XRCE agent on connected stronger microprocessor.
Support for multiple DDS implementations, chosen at runtime | <span class="status_flag">&#10003;</span> | Support is possible in principle, but at compile-time only.
Common core client library that is wrapped by language-specific libraries | <span class="status_flag">&#10003;</span> | Use of the client support library [rcl](https://github.com/ros2/rcl/) from ROS 2 as-is. The [rclc](https://github.com/micro-ROS/rclc) package provides convenience functions and an executor for use of rcl+rclc as client library for the C programming language.
Publish/subscribe over topics | <span class="status_flag">&#10003;</span> | Available, but only fixed-size message types supported to avoid dynamic memory allocations.
Clients and services | <span class="status_flag">&#10003;</span> | Available, but only fixed-size message types supported to avoid dynamic memory allocations.
ROS 1 -- ROS 2 communication bridge | <span class="status_flag">&ndash;</span> | Not applicable, standard ROS 1 -- ROS 2 bridge can be used via micro-ROS Agent on a stronger microprocessor to communicate with micro-ROS nodes.
Actions | <span class="status_flag">&#9675;</span> | *Not yet implemented.*
Parameters | <span class="status_flag">&#10003;</span> | *To be implemented soon in rclc.*
Node Graph | <span class="status_flag">&#10003;</span> | Available as in ROS 2.
Discovery | <span class="status_flag">&#10003;</span> | Available as in ROS 2.
Quality of service settings for handling non-ideal networks | <span class="status_flag">&#10003;</span> | Two QoS semantics, reliable and best-effort semantics, are provided and can be set at compile-time.
Inter- and intra-process communication using the same API | <span class="status_flag">&#10003;</span> | No shared-memory interprocess communication on the MCU available, but all communication is performed via the micro-ROS-Agent running on a connected microprocessor. *Efficient shared-memory communication on the MCU is considered as an important feature for future releases.*
Composition of node components at compile-, link- or dlopen-time | <span class="status_flag">&#10003;</span> | Composition at compile-time only. Composition at runtime would depend highly on the RTOS.
Support for nodes with managed lifecycles | <span class="status_flag">&#10003;</span> | The [rclc_lifecycle](https://github.com/micro-ROS/rclc/blob/master/rclc_lifecycle/) package provides an `rclc_lifecycle_node` type which bundles an rcl node with the lifecycle state machine as well as corresponding convenience functions.
DDS-Security support | <span class="status_flag">&#10003;</span> | DDS security is supported at micro-ROS-Agent. *Security mechanisms in Micro XRCE-DDS are planned for future releases.*
Command-line introspection tools using an extensible framework | <span class="status_flag">&#10003;</span> | From a remote microprocessor all standard ROS 2 tools can be used to introspect the micro-ROS nodes on an MCU. Micro-ROS nodes appear as ROS 2 nodes (by the micro-ROS-Agent).
Launch system for coordinating multiple nodes | <span class="status_flag">&#10003;</span> | No launch system for the micro-ROS nodes on an MCU available. Such a system would depend highly on the RTOS. The system-modes concept developed with micro-ROS allows runtime configuration/orchestration of ROS 2 and micro-ROS nodes together.
Namespace support for nodes and topics | <span class="status_flag">&#10003;</span> | Available as in ROS 2.
Static remapping of ROS names | <span class="status_flag">&#10003;</span> | *Should be available if passed as argument via standard rcl API -- to be checked.*
Demos of an all-ROS 2 mobile robot | <span class="status_flag">&#10003;</span> | Demos of several ROS 2 + micro-ROS robots available. See [https://micro-ros.github.io/docs/tutorials/demos/](https://micro-ros.github.io/docs/tutorials/demos/).
Support for real-time code | <span class="status_flag">&#10003;</span> | The [rclc Executor](https://github.com/micro-ROS/rclc/tree/master/rclc) provides mechanisms for implementing real-time-critical applications with micro-ROS.
Support for "bare-metal" microcontrollers | <span class="status_flag">&#10003;</span> | Bringing ROS 2 onto MCUs is all that micro-ROS is about. A crucial difference to this requirement from the early design phase of ROS 2 is that micro-ROS assumes an RTOS (e.g., [FreeRTOS](https://www.freertos.org/), [Zephyr](https://www.zephyrproject.org/), or [NuttX](http://nuttx.apache.org/)).
IDL | <span class="status_flag">&#10003;</span> | Same message IDL as with ROS 2, but use of resource-optimized CDR serialization implementation named [Micro-CDR](https://github.com/eProsima/Micro-CDR).
Build system | <span class="status_flag">&#10003;</span> | Build systems of NuttX, FreeRTOS, and Zephyr are integrated with colcon. Furthermore, micro-ROS is provided as a component for ESP-IDF also as a standalone Zephyr module. The build system is likely the most fragile part of micro-ROS w.r.t. the long-term maintenance, due to the many dependencies.
Continuous Integration | <span class="status_flag">&#10003;</span> | Currently, the CI for micro-ROS is distributed to GitHub and Gitlab. *Until the end of 2020, all CI should be moved migrated completely to the new CI actions of GitHub.* Please note that those packages that are released for standard ROS 2 are also built and tested on [build.ros2.org](http://build.ros2.org/).
Documentation | <span class="status_flag">&#10003;</span> | High-level documentation at [micro-ros.github.io](https://micro-ros.github.io/). For detailed information please consult the README.md files in the relevant micro-ROS repositories at [github.com/micro-ROS/](https://github.com/micro-ROS/).
Logging | <span class="status_flag">&#10003;</span> | *Could be available as part of the standard logging mechanism in principle but not supported by Micro-XRCE-DDS due to dynamic message size. To be checked ...*
Support of rate and sleep with system clock | <span class="status_flag">&#10003;</span> | rcl timers use POSIX API. Tested successfully on NuttX, but the resolution is very low. A higher resolution could be achieved with hardware timers -- which highly depends on the MCU and possibly the RTOS. *This feature requires further investigation.*
Support for simulation time | <span class="status_flag">&#10003;</span> | *Might be supported out of the box, but needs to be checked.* We consider HIL setups with simulation time to be corner cases.
