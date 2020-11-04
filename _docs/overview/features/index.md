---
title: Features Overview
permalink: /docs/overview/features/
---

<section style="display: grid; grid-template-columns: 1fr 9fr;">
 <h1 style="writing-mode: vertical-lr; text-align: center; color: #C3DBEE;">
  ROS Interoperability
 </h1>
 <div>
  <h2>Support for major ROS concepts</h2>
  <p>Micro-ROS brings all major core concepts such as nodes, publish/subscribe (topics), client/service, parameters, node lifecycle on microcontrollers (MCU). The client API of micro-ROS (in the C programming language) is based on the standard ROS 2 Client Support Library (rcl) and a set of extensions and convenience functions (rclc).</p>

  <h2>Seamless integration with ROS 2</h2>
  <p>The micro-ROS agent connects micro-ROS nodes (i.e. components) on MCUs seamlessly with standard ROS 2 systems. This allows accessing micro-ROS nodes with the known ROS 2 tools and APIs just as normal ROS nodes.</p>
 </div>
</section>

<section style="display: grid; grid-template-columns: 1fr 9fr; margin-top: 3em;">
 <h1 style="writing-mode: vertical-lr; text-align: center; color: #C3DBEE;">
  Deeply Embedded
 </h1>
 <div>
  <h2>Multi-RTOS support</h2>
  <p>Micro-ROS supports three important open-source real-time operating sytems (RTOS): FreeRTOS, Zephyr, and NuttX. It can be ported on any RTOS that comes with a POSIX interface.</p>

  <h2>MCU-optimized middleware</h2>
  <p>Micro XRCE-DDS by eProsima meets all requirements for a middleware for deeply embedded systems. That is why micro-ROS has been one of the applications for this implementation of the new DDS for Extremely Resource Constrained Environments (XRCE) standard. For the integration with the ROS middleware interface (rmw) in the micro-ROS stack, static memory pools were introduced to avoid dynamic memory allocations at runtime.</p>

  <h2>Support for major transports</h2>
  <p>By the Micro XRCE-DDS middleware, micro-ROS comes with built-in support for serial transports, UDP over Ethernet, Wi-Fi, and 6LoWPAN, and Bluetooth. The Micro XRCE-DDS source code provides templates for implementing support for further transports.</p>

  <h2>MCU-optimized ROS client API</h2>
  <p>The combination rcl+rclc is optimized for MCUs. After an initialization phase, it can be used without any dynamic memory allocations. The rclc package provides advanced execution mechanisms allowing implementing well-proven scheduling patterns from embedded systems engineering.</p>  
 </div>
</section>

<section style="display: grid; grid-template-columns: 1fr 9fr; margin-top: 3em;">
 <h1 style="writing-mode: vertical-lr; text-align: center; color: #C3DBEE;">
  Tooling
 </h1>
 <div>
  <h2>Flexible build system</h2>
  <p>Micro-ROS integrates the RTOS-specific build systems into few scripts that are provided as a ROS 2 package. Therefore, ROS developers can use their usual command line tools. In addition, micro-ROS provides selected integrations with RTOS-specific tool chains (e.g., for ESP-IDF and Zephyr).</p>
  
  <h2>Benchmarking Tools</h2>
  <p>To optimize micro-ROS-based applications to the MCU hardware, micro-ROS provides benchmarking tools. This allows us to check memory usage, CPU time consumption and general performance.</p>
 </div>
</section>

<section style="display: grid; grid-template-columns: 1fr 9fr; margin-top: 3em;">
 <h1 style="writing-mode: vertical-lr; text-align: center; color: #C3DBEE;">
  Openness
 </h1>
 <div>
  <h2>Permissive license</h2>
  <p>The micro-ROS stack, including the Micro XRCE-DDS middleware, comes under the same permissive license as ROS 2, which is Apache License 2.0. (When building projects with micro-ROS, please take into account the license(s) of the underlying RTOS.)</p>
  
  <h2>Community and Support</h2>
  <p>Micro-ROS is developed by a constantly growing, self-organized community backed by the Embedded Working Group, a formal ROS 2 Working Group. The community shares entry level tutorials, provides support via Slack and GitHub, and meets in public Working Group video-calls on a monthly basis. As a matter of course, commercial support is provided for the Micro XRCE-DDS by eProsima.</p>
  
  <h2>High maintainability</h2>
  <p>Micro-ROS is made up of well-established components: Famous open-source RTOSs, a standardized middleware, and the standard ROS 2 Client Support Library (rcl). In this way, the amount of micro-ROS-specific code was minimized for long-term maintainability.</p>

  <h2>Interoperability with other standards</h2>
  <p>The micro-ROS stack preserves the modularity of the standard ROS 2 stack. It can be used with a custom middleware layer - and thus standard - or a custom ROS client library. Furthermore, in the context of micro-ROS, we have developed the FIROS2 integration services, which connects the Robot Operating System with the FIWARE Context Broker by the NGSIv2 (Next Generation Service Interface) standard.</p>
  
 </div>
</section>

