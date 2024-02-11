---
title: Features and Architecture
permalink: /docs/overview/features/
redirect_from:
  - /docs/
  - /docs/overview/
  - /architecture/
---

<script src="https://code.jquery.com/jquery-1.12.4.min.js"></script>
<script>
function updateVisibilityOfFeatureDescriptions() {
  $('.feature').not('feature_is_active').find('.three_dots').show();
  $('.feature').not('feature_is_active').find('.feature_description').slideUp(500);
  $('.feature_is_active').find('.three_dots').hide();
  $('.feature_is_active').find('.feature_description').slideDown(500);
}

$(document).ready( function() {
  $('.feature_description').hide();
  updateVisibilityOfFeatureDescriptions();

  $('.feature').click( function() {
    if ($(this).hasClass("feature_is_active")) {
      $(this).removeClass("feature_is_active");
    } else {
      $('.feature').removeClass("feature_is_active");
      $(this).addClass("feature_is_active");
    }
    updateVisibilityOfFeatureDescriptions();
  });
});
</script>

<style>
  .three_dots {
    color: #BBBBBB;
  }
  .feature_title {
    font-weight: bold;
    margin: 8pt 0 2pt 0;
  }
  .feature_description {
    margin-left: 3em;
  }
  .feature_description > p {
    margin: 0 0 2pt 0;
  }
</style>

Micro-ROS offers **seven key features** that make it ready for use in your microcontroller-based robotic project:

<div class="feature feature_is_active">
 <div class="feature_title">&#10004; Microcontroller-optimized client API supporting all major ROS concepts<span class="three_dots"> (...)</span></div>
 <div class="feature_description">
  <p>Micro-ROS brings all major core concepts such as nodes, publish/subscribe, client/service, node graph, lifecycle, etc. onto microcontrollers (MCU). The client API of micro-ROS (in the C programming language) is based on the standard ROS 2 Client Support Library (rcl) and a set of extensions and convenience functions (rclc).</p>
  <p>The combination rcl+rclc is optimized for MCUs. After an initialization phase, it can be used without any dynamic memory allocations. The rclc package provides advanced execution mechanisms allowing implementing well-proven scheduling patterns from embedded systems engineering.</p>
 </div>
</div>

<div class="feature">
 <p class="feature_title">&#10004; Seamless integration with ROS 2<span class="three_dots"> (...)</span></p>
 <div class="feature_description">
  <p>The micro-ROS agent connects micro-ROS nodes (i.e. components) on MCUs seamlessly with standard ROS 2 systems. This allows accessing micro-ROS nodes with the known ROS 2 tools and APIs just as normal ROS nodes.</p>
 </div>
</div>

<div class="feature">
 <p class="feature_title">&#10004; Extremely resource-constrained but flexible middleware<span class="three_dots"> (...)</span></p>
 <div class="feature_description">
  <p>Micro XRCE-DDS by eProsima meets all requirements for a middleware for deeply embedded systems. That is why micro-ROS has been one of the applications for this implementation of the new DDS for Extremely Resource Constrained Environments (XRCE) standard. For the integration with the ROS middleware interface (rmw) in the micro-ROS stack, static memory pools were introduced to avoid dynamic memory allocations at runtime.</p>
  <p>The middleware comes with built-in support for serial transports, UDP over Ethernet, Wi-Fi, and 6LoWPAN, and Bluetooth. Furthermore, the Micro XRCE-DDS source code provides templates for implementing support for further transports.</p>
 </div>
</div>

<div class="feature">
 <p class="feature_title">&#10004; Multi-RTOS support with generic build system<span class="three_dots"> (...)</span></p>
 <div class="feature_description">
  <p>Micro-ROS supports three popular open-source real-time operating sytems (RTOS): FreeRTOS, Zephyr, and NuttX. It can be ported on any RTOS that comes with a POSIX interface.</p>
  <p>The RTOS-specific build systems are integrated into few generic setup scripts, which are provided as a ROS 2 package. Therefore, ROS developers can use their usual command line tools. In addition, micro-ROS provides selected integrations with RTOS-specific tool chains (e.g., for ESP-IDF and Zephyr).</p>
 </div>
</div>

<div class="feature">
 <p class="feature_title">&#10004; Permissive license<span class="three_dots"> (...)</span></p>
 <div class="feature_description">
  <p>Micro-ROS comes under the same permissive license as ROS 2, which is Apache License 2.0. This applies to the micro-ROS client library, the middleware layer, and tools.</p>
  <p>When creating a project with an underlying RTOS, please take into account the license(s) of the RTOS project or vendor as further explained on the <a href="../license/">license</a> page.</p>
 </div>
</div>

<div class="feature">
 <p class="feature_title">&#10004; Vibrant community and ecosystem<span class="three_dots"> (...)</span></p>
 <div class="feature_description">
  <p>Micro-ROS is developed by a constantly growing, self-organized community backed by the Embedded Working Group, a formal ROS 2 Working Group. The community shares entry level tutorials, provides support via Slack and GitHub, and meets in public Working Group video-calls on a monthly basis. As a matter of course, commercial support is provided for the Micro XRCE-DDS by eProsima.</p>
  <p>This community also create tools around micro-ROS. For example, to optimize micro-ROS-based applications to the MCU hardware, specific benchmarking tools have been developed. These allow checking memory usage, CPU time consumption and general performance.</p>
 </div>
</div>

<div class="feature">
 <p class="feature_title">&#10004; Long-term maintainability and interoperability<span class="three_dots"> (...)</span></p>
 <div class="feature_description">
  <p>Micro-ROS is made up of well-established components: Famous open-source RTOSs, a standardized middleware, and the standard ROS 2 Client Support Library (rcl). In this way, the amount of micro-ROS-specific code was minimized for long-term maintainability. At the same time, the micro-ROS stack preserves the modularity of the standard ROS 2 stack. Micro-ROS can be used with a custom middleware layer - and thus standard - or a custom ROS client library.</p>
  <p>Furthermore, by the <a href="https://soss.docs.eprosima.com/">System-Of-Systems Synthesizer</a> (SOSS), a fast and lightweight <a href="https://www.omg.org/spec/DDS-XTypes">OMG DDS-XTYPES standard</a> integration tool, further middleware protocols can be connected. For example, we have developed the SOSS-FIWARE and SOSS-ROS2 System-Handles, which connect ROS 2 and micro-ROS with the <a href="https://www.fiware.org/">FIWARE Context Broker</a> by the NGSIv2 (Next Generation Service Interface) standard by leveraging the integration capabilities of the SOSS core.</p>
 </div>
</div>

## Layered and Modular Architecture

Micro-ROS follows the [ROS 2 architecture](https://docs.ros.org/en/rolling/Concepts/Advanced/About-Internal-Interfaces.html) and makes use of its middleware pluggability to use [DDS-XRCE](https://www.omg.org/spec/DDS-XRCE/), which is optimized for microcontrollers. Moreover, it uses POSIX-based RTOS (FreeRTOS, Zephyr, or NuttX) instead of Linux.

<img src="/img/micro-ROS_architecture.png" style="display: block; margin: auto; width: 100%; max-width: 500px;"/>

Dark blue components are developed specifically for micro-ROS. Light blue components are taken from the standard ROS 2 stack. We seek to contribute as much code back to the ROS 2 mainline codebase as possible.
