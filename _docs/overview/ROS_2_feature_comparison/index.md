---
title: ROS 2 Feature Comparison
permalink: /docs/overview/ROS_2_feature_comparison/
---

Comparison of micro-ROS features with ROS 2 features. The following list has been compiled from [https://index.ros.org/doc/ros2/Features/](https://index.ros.org/doc/ros2/Features/) and [https://index.ros.org/doc/ros2/Roadmap/](https://index.ros.org/doc/ros2/Roadmap/), and the features have been organized into sub-tables according to the macrocategories defined in the [Features and Architecture page](https://micro-ros.github.io//docs/overview/features/).

<style>
  .status_flag {
    font-size: 150%;
    font-weight: bold;
  }
</style>

### Microcontroller-optimized client API supporting all major ROS concepts

ROS 2 Feature | | Availability in micro-ROS
-- | -- | --
Common core client library that is wrapped by language-specific libraries | <span class="status_flag">&#10003;&#8728;</span> | Use of the client support library [rcl](https://github.com/ros2/rcl/) from ROS 2 as-is. The [rclc](https://github.com/ros2/rclc) package provides convenience functions and an executor for use of rcl+rclc as client library for the C programming language. *Roadmap: migrate all functionalities to the rclc, so as to make it an independent abstraction layer on top of the rcl that serves as user's API.*
Composition of node components at compile-, link- or dlopen-time | <span class="status_flag">&#10003;</span> | Composition at compile-time only. Composition at runtime would depend highly on the RTOS.
Support for nodes with managed lifecycles | <span class="status_flag">&#10003;</span> | The [rclc_lifecycle](https://github.com/ros2/rclc/blob/master/rclc_lifecycle/) package provides an `rclc_lifecycle_node` type which bundles an rcl node with the lifecycle state machine as well as corresponding convenience functions.

### Seamless integration with ROS 2

ROS 2 Feature | | Availability in micro-ROS
-- | -- | --
Publish/subscribe over topics | <span class="status_flag">&#10003;</span> | Available, but only fixed-size message types supported to avoid dynamic memory allocations.
Clients and services | <span class="status_flag">&#10003;</span> | Available, but only fixed-size message types supported to avoid dynamic memory allocations.
ROS 1 -- ROS 2 communication bridge | <span class="status_flag">&#10003;</span> | Standard ROS 1 -- ROS 2 bridge or [SOSS-mediate bridge](https://soss.docs.eprosima.com/en/latest/getting_started.html#example-ros1-ros2-communication) can be used via micro-ROS Agent to communicate with micro-ROS nodes.
Actions | <span class="status_flag">&#8728;</span> | *To be implemented soon in rclc.*
Parameters | <span class="status_flag">&#8728;</span> | *To be implemented soon in rclc.*
Node Graph | <span class="status_flag">&#10003;</span> | Available as in ROS 2.
Discovery | <span class="status_flag">&#10003;&#43;</span> | Discovery between entities available as in ROS 2. Further discovery mechanism available for the Clients to discover Agents on the network.
Inter- and intra-process communication using the same API | <span class="status_flag">&#8331;</span> | No shared-memory interprocess communication on the MCU available, but all communication is performed via the micro-ROS-Agent running on a connected microprocessor. Possibility to leverage multi-thread functionalities offered by RTOS. *Efficient shared-memory communication on the MCU is considered as an important feature for future releases.*
Command-line introspection tools using an extensible framework | <span class="status_flag">&#10003;</span> | Thanks to graph support, standard ROS 2 tools can be used to introspect the topology of the ROS 2 dataspace, via the Agent, from a microprocessor running a micro-ROS node . At the same time, standard ROS 2 nodes can fetch information regarding the micro-ROS entities present on the network.
Launch system for coordinating multiple nodes | <span class="status_flag">&#8331;</span> | No launch system for the micro-ROS nodes on an MCU available. Such a system would depend highly on the RTOS. The system-modes concept developed with micro-ROS allows runtime configuration/orchestration of ROS 2 and micro-ROS nodes together.
Namespace support for nodes and topics | <span class="status_flag">&#10003;</span> | Available as in ROS 2.
Static remapping of ROS names | <span class="status_flag">&#8727;</span> | *Should be available if passed as argument via standard rcl API -- to be checked.*
Support of rate and sleep with system clock | <span class="status_flag">&#8727;</span> | rcl timers use POSIX API. Tested successfully on NuttX, but the resolution is very low. A higher resolution could be achieved with hardware timers -- which highly depends on the MCU and possibly the RTOS. *This feature requires further investigation.*
Support for simulation time | <span class="status_flag">&#8727;</span> | *Might be supported out of the box, but needs to be checked.* We consider HIL setups with simulation time to be corner cases.

### Extremely resource-constrained but flexible middleware

ROS 2 Feature | | Availability in micro-ROS
-- | -- | --
Transport and serialization over DDS-XRCE and DDS| <span class="status_flag">&#10003;&#43;</span> | Available transports: UDP, serial (UART) and custom as enabled by [Micro XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS). Serialization between Client and Agent provided by [Micro-CDR](https://github.com/eProsima/Micro-CDR) and between Agent to standard DDS by [Fast-CDR](https://github.com/eProsima/Fast-CDR).
Support for multiple DDS implementations, chosen at runtime | <span class="status_flag">&#10003;</span> | Support via the Micro XRCE-DDS Agent is possible in principle, but at compile-time only.
Quality of service settings for handling non-ideal networks | <span class="status_flag">&#10003;&#43;</span> | For communication over the DDS-XRCE wire protocol, two QoS semantics, reliable and best-effort, are provided and can be set at compile-time. As for communication with the ROS 2 dataspace, micro-ROS entities can benefit from the whole set of QoS allowed by DDS when created [by Reference](https://micro-ros.github.io/docs/tutorials/core/create_dds_entities_by_ref/).
DDS-Security support | <span class="status_flag">&#10003;&#45;</span> | Security is not yet supported in the communication process between the Client and the Agent. However, the micro-ROS Agent can benefit from Fast DDS security capabilities during the creation of DDS entities. *Roadmap: Implementation of security mechanisms in Micro XRCE-DDS are planned for future releases.*
IDL | <span class="status_flag">&#10003;&#43;</span> | micro-ROS supports the same IDL types as ROS 2. Generation of C code from IDLs as handled by the Client is performed by the [Micro-XRCE-DDS-Gen](https://github.com/eProsima/Micro-XRCE-DDS-Gen) library, whereas generation of the C++ types handled by the Agent is handled by [Fast-DDS-Gen](https://github.com/eProsima/Fast-DDS-Gen).
Logging | <span class="status_flag">&#8727;</span> | *Could be available as part of the standard logging mechanism in principle but not supported by Micro-XRCE-DDS due to dynamic message size. To be checked ...*

### Multi-RTOS support with generic build system

Feature | | Availability in micro-ROS
-- | -- | --
Build system | <span class="status_flag">&#10003;</span> | micro-ROS provides two ways of building a micro-ROS application. The first uses the [micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup) tool integrated in a ROS 2 workspace. With this approach, the build systems of NuttX, FreeRTOS, and Zephyr are integrated with colcon. The other provides micro-ROS as a component for external development frameworks (e.g., ESP-IDF and Zephyr build system).
Supported hardware | <span class="status_flag">&#10003;</span> | micro-ROS officially supports four boards. For the moment, all official ports are based on the STM32 series from ST and on the ESP32 from Espressif. Find more info [here](https://micro-ros.github.io/docs/overview/hardware/). More ports have been carried out by users, check the [complete list](https://github.com/micro-ROS/micro_ros_setup#supported-platforms).
Supported Operating Systems | <span class="status_flag">&#10003;</span> | micro-ROS is supported by the RTOSes FreeRTOS, Zephyr, NuttX, in addition to Linux and Windows.

### micro-ROS specific features

Feature | | Availability in micro-ROS
-- | -- | --
Demos of an all-ROS 2 mobile robot | <span class="status_flag">&#10003;</span> | Demos of several ROS 2 + micro-ROS robots available. See [https://micro-ros.github.io/docs/tutorials/demos/](https://micro-ros.github.io/docs/tutorials/demos/).
Support for real-time code | <span class="status_flag">&#10003;</span> | Real-time behaviour is key to micro-ROS typical usages. The [rclc Executor](https://github.com/ros2/rclc/tree/master/rclc) provides mechanisms for implementing real-time-critical applications. At lower levels, the Micro XRCE-DDS library exhibits real-timeness and determinism for being dynamic memory free and for providing functions to perform tasks within well-defined periods of time.
Support for "bare-metal" microcontrollers | <span class="status_flag">&#10003;</span> | Bringing ROS 2 onto MCUs is all that micro-ROS is about. The standard approach to micro-ROS assumes an RTOS underneath (e.g., [FreeRTOS](https://www.freertos.org/), [Zephyr](https://www.zephyrproject.org/), or [NuttX](http://nuttx.apache.org/)). Recent developments aim at loosening this requirement, with the integration into [Arduino IDE](https://github.com/micro-ROS/micro_ros_arduino) being a first step towards true micro-ROS bare-metal support.
Continuous Integration | <span class="status_flag">&#10003;&#8728;</span> | Currently, the CI for micro-ROS is distributed to GitHub and GitLab. *Until the end of 2020, all CI should be moved migrated completely to the new CI actions of GitHub.* Please note that those packages that are released for standard ROS 2 are also built and tested on [build.ros2.org](http://build.ros2.org/).
Documentation | <span class="status_flag">&#10003;</span> | High-level documentation at [micro-ros.github.io](https://micro-ros.github.io/). For detailed information please consult the README.md files in the relevant micro-ROS repositories at [github.com/micro-ROS/](https://github.com/micro-ROS/). For information on the middleware implementation, take a look at the [Micro XRCE-DDS documentation](https://micro-xrce-dds.docs.eprosima.com/en/latest/).
Peer-to-peer functionality | <span class="status_flag">&#10003;&#8728;</span> | Prototypical peer-to-peer functionality implemented over broadcast. No QoS available for the moment. *Roadmap: improve prototype to achieve true point-to-point connection.*
Memory footprint | <span class="status_flag">&#10003;</span> | A comprehensive profiling of the memory consumption of typical micro-ROS applications can be found [here](https://micro-ros.github.io/docs/concepts/benchmarking/memo_prof/).

Below, you can find the legend of the symbols used in the tables above.

Symbols legend |
-- | --
<span class="status_flag">&#10003;</span> | Available.
<span class="status_flag">&#10003;&#43;</span> | Available both on Agent-DDS and Client-Agent sides of the communication.
<span class="status_flag">&#10003;&#45;</span> | Available on Agent-DDS side of the communication but not on Client-Agent side.
<span class="status_flag">&#10003;&#8728;</span> | Available with some WIP feature.
<span class="status_flag">&#8728;</span> | To be implemented soon.
<span class="status_flag">&#8727;</span> | Further investigation required.
<span class="status_flag">&#8331;</span> | Currently unavailable.
