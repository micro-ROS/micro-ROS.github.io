---
title: Features Overview
permalink: /docs/overview/features/
redirect_from:
  - /docs/
  - /docs/overview/
---

<script src="https://code.jquery.com/jquery-1.12.4.min.js"></script>
<script>
$(document).ready( function() {
  $('.feature_description').hide();
  $('.feature').click( function() {
    var featureDescription = $(this).children('.feature_description');
    if (featureDescription.is(":visible")) {
      featureDescription.slideToggle(500);
      setTimeout(function() {
        featureDescription.prev('.feature_teaser').slideToggle(0);
      }, 500);
    } else {
      featureDescription.prev('.feature_teaser').slideToggle(0);
      featureDescription.slideToggle(500);
    }
  });
});
</script>

<style>
  .feature_teaser {
    color: #AAAAAA;
    margin-left: 1.6em;
  }
  .feature_description {
    text-align: justify;
    margin-left: 1.6em;
  }
  .feature_teaser ~ p {
    margin: 0 0 5pt 0;
  }
  .feature_description ~ p {
    margin: 0 0 5pt 0;
  }
</style>

<div class="feature">
 <h3 class="feature_name">&#9745; Microcontroller-optimized client API supporting all major ROS concepts</h3>
 <div class="feature_teaser">
  <p>Micro-ROS brings all major core concepts ...</p>
 </div>
 <div class="feature_description">
  <p>Micro-ROS brings all major core concepts such as nodes, publish/subscribe, client/service, node graph, lifecycle, etc. onto microcontrollers (MCU). The client API of micro-ROS (in the C programming language) is based on the standard ROS 2 Client Support Library (rcl) and a set of extensions and convenience functions (rclc).</p>
  <p>The combination rcl+rclc is optimized for MCUs. After an initialization phase, it can be used without any dynamic memory allocations. The rclc package provides advanced execution mechanisms allowing implementing well-proven scheduling patterns from embedded systems engineering.</p>
 </div>
</div>

<div class="feature">
 <h3 class="feature_name">&#9745; Seamless integration with ROS 2</h3>
 <div class="feature_teaser">
  <p>The micro-ROS agent connects micro-ROS nodes ...</p>
 </div>
 <div class="feature_description">
  <p>The micro-ROS agent connects micro-ROS nodes (i.e. components) on MCUs seamlessly with standard ROS 2 systems. This allows accessing micro-ROS nodes with the known ROS 2 tools and APIs just as normal ROS nodes.</p>
 </div>
</div>

<div class="feature">
 <h3 class="feature_name">&#9745; Multi-RTOS support with generic build system</h3>
 <div class="feature_teaser">
  <p>Micro-ROS supports three popular open-source ...</p>
 </div>
 <div class="feature_description">
  <p>Micro-ROS supports three popular open-source real-time operating sytems (RTOS): FreeRTOS, Zephyr, and NuttX. It can be ported on any RTOS that comes with a POSIX interface.</p>
  <p>The RTOS-specific build systems are integrated into few generic setup scripts, which are provided as a ROS 2 package. Therefore, ROS developers can use their usual command line tools. In addition, micro-ROS provides selected integrations with RTOS-specific tool chains (e.g., for ESP-IDF and Zephyr).</p>
 </div>
</div>

<div class="feature">
 <h3 class="feature_name">&#9745; Extremely resource-constrained but flexible middleware</h3>
 <div class="feature_teaser">
  <p>Micro XRCE-DDS by eProsima meets all requirements ...</p>
 </div>
 <div class="feature_description">
  <p>Micro XRCE-DDS by eProsima meets all requirements for a middleware for deeply embedded systems. That is why micro-ROS has been one of the applications for this implementation of the new DDS for Extremely Resource Constrained Environments (XRCE) standard. For the integration with the ROS middleware interface (rmw) in the micro-ROS stack, static memory pools were introduced to avoid dynamic memory allocations at runtime.</p>
  <p>The middleware comes with built-in support for serial transports, UDP over Ethernet, Wi-Fi, and 6LoWPAN, and Bluetooth. Furthermore, the Micro XRCE-DDS source code provides templates for implementing support for further transports.</p>
 </div>
</div>

<div class="feature">
 <h3 class="feature_name">&#9745; Permissive license</h3>
 <div class="feature_teaser">
  <p>Micro-ROS comes under the same permissive ...</p>
 </div>
 <div class="feature_description">
  <p>Micro-ROS comes under the same permissive license as ROS 2, which is Apache License 2.0. This applies to the micro-ROS client library, the middleware layer, and tools.</p>
  <p>When creating a project with an underlying RTOS, please take into account the license(s) of the RTOS project or vendor.</p>
 </div>
</div>

<div class="feature">
 <h3 class="feature_name">&#9745; Vibrant community and ecosystem</h3>
 <div class="feature_teaser">
  <p>Micro-ROS is developed by a constantly growing ...</p>
 </div>
 <div class="feature_description">
  <p>Micro-ROS is developed by a constantly growing, self-organized community backed by the Embedded Working Group, a formal ROS 2 Working Group. The community shares entry level tutorials, provides support via Slack and GitHub, and meets in public Working Group video-calls on a monthly basis. As a matter of course, commercial support is provided for the Micro XRCE-DDS by eProsima.</p>
  <p>This community also create tools around micro-ROS. For example, to optimize micro-ROS-based applications to the MCU hardware, specific benchmarking tools have been developed. These allow checking memory usage, CPU time consumption and general performance.</p>
 </div>
</div>

<div class="feature">
 <h3 class="feature_name">&#9745; Long-term maintainability and interoperability</h3>
 <div class="feature_teaser">
  <p>Micro-ROS is made up of well-established components ...</p>
 </div>
 <div class="feature_description">
  <p>Micro-ROS is made up of well-established components: Famous open-source RTOSs, a standardized middleware, and the standard ROS 2 Client Support Library (rcl). In this way, the amount of micro-ROS-specific code was minimized for long-term maintainability. At the same time, the micro-ROS stack preserves the modularity of the standard ROS 2 stack. Micro-ROS can be used with a custom middleware layer - and thus standard - or a custom ROS client library.</p>
  <p>Furthermore, by the [System-Of-Systems Synthesizer](https://soss.docs.eprosima.com/) (SOSS), a fast and lightweight [OMG DDS-XTYPES standard](https://www.omg.org/spec/DDS-XTypes) integration tool, further middleware protocols can be connected. For example, we have developed the SOSS-FIWARE and SOSS-ROS2 System-Handles, which connect ROS 2 and micro-ROS with the [FIWARE Context Broker](https://www.fiware.org/) by the NGSIv2 (Next Generation Service Interface) standard by leveraging the integration capabilities of the SOSS core.</p>
 </div>
</div>