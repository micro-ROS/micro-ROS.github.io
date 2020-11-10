---
title: Features Overview
permalink: /docs/overview/features/
---

TODO(ralph-lange): Add few introductory sentences and a nice graphics. Then make this the overview page.



### Microcontroller-optimized client API supporting all major ROS concepts

Micro-ROS brings all major core concepts such as nodes, publish/subscribe, client/service, node graph, lifecycle, etc. onto microcontrollers (MCU). The client API of micro-ROS (in the C programming language) is based on the standard ROS 2 Client Support Library (rcl) and a set of extensions and convenience functions (rclc).

The combination rcl+rclc is optimized for MCUs. After an initialization phase, it can be used without any dynamic memory allocations. The rclc package provides advanced execution mechanisms allowing implementing well-proven scheduling patterns from embedded systems engineering.

### Seamless integration with ROS 2

The micro-ROS agent connects micro-ROS nodes (i.e. components) on MCUs seamlessly with standard ROS 2 systems. This allows accessing micro-ROS nodes with the known ROS 2 tools and APIs just as normal ROS nodes.

### Multi-RTOS support with generic build system
Micro-ROS supports three popular open-source real-time operating sytems (RTOS): FreeRTOS, Zephyr, and NuttX. It can be ported on any RTOS that comes with a POSIX interface.

The RTOS-specific build systems are integrated into few generic setup scripts, which are provided as a ROS 2 package. Therefore, ROS developers can use their usual command line tools. In addition, micro-ROS provides selected integrations with RTOS-specific tool chains (e.g., for ESP-IDF and Zephyr).

### Extremely resource-constrained but flexible middleware

Micro XRCE-DDS by eProsima meets all requirements for a middleware for deeply embedded systems. That is why micro-ROS has been one of the applications for this implementation of the new DDS for Extremely Resource Constrained Environments (XRCE) standard. For the integration with the ROS middleware interface (rmw) in the micro-ROS stack, static memory pools were introduced to avoid dynamic memory allocations at runtime.

The middleware comes with built-in support for serial transports, UDP over Ethernet, Wi-Fi, and 6LoWPAN, and Bluetooth. Furthermore, the Micro XRCE-DDS source code provides templates for implementing support for further transports.

### Permissive license

The micro-ROS stack, including the Micro XRCE-DDS middleware, comes under the same permissive license as ROS 2, which is Apache License 2.0. (When building projects with micro-ROS, please take into account the license(s) of the underlying RTOS.)

### Vibrant community and ecosystem

Micro-ROS is developed by a constantly growing, self-organized community backed by the Embedded Working Group, a formal ROS 2 Working Group. The community shares entry level tutorials, provides support via Slack and GitHub, and meets in public Working Group video-calls on a monthly basis. As a matter of course, commercial support is provided for the Micro XRCE-DDS by eProsima.

This community also create tools around micro-ROS. For example, to optimize micro-ROS-based applications to the MCU hardware, specific benchmarking tools have been developed. These allow checking memory usage, CPU time consumption and general performance.

### Long-term maintainability and interoperability

Micro-ROS is made up of well-established components: Famous open-source RTOSs, a standardized middleware, and the standard ROS 2 Client Support Library (rcl). In this way, the amount of micro-ROS-specific code was minimized for long-term maintainability. At the same time, the micro-ROS stack preserves the modularity of the standard ROS 2 stack. Micro-ROS can be used with a custom middleware layer - and thus standard - or a custom ROS client library.

Furthermore, by the [System-Of-Systems Synthesizer](https://soss.docs.eprosima.com/) (SOSS), a fast and lightweight [OMG DDS-XTYPES standard](https://www.omg.org/spec/DDS-XTypes) implementation, further middleware protocols can be connected. For example, we have developed the SOSS-FIWARE and SOSS-ROS2 System-Handles, which connect ROS 2 and micro-ROS with the [FIWARE Context Broker](https://www.fiware.org/) by the NGSIv2 (Next Generation Service Interface) standard by leveraging the integration capabilities of the SOSS core.