---
title: First micro-ROS Application on an RTOS
permalink: /docs/tutorials/core/first_application_rtos/
---

After you have completed the [First micro-ROS application on Linux tutorial](../first_application_linux), you are now ready to flash a microcontroller with this application based on a Real-Time Operating System (RTOS).

Micro-ROS currently supports three different RTOS, namely NuttX, FreeRTOS, and Zephyr. Of course, the micro-ROS-related sections of the application code are independent of the underlying RTOS. Also, the basic tooling is the same as we have integrated the RTOS tools with the ROS 2 meta build system colcon. However, there are subtle differences in the configuration and the definition of the executables between the three RTOS. Therefore, for this tutorial, please decide for one RTOS to use:

<table style="border:none;">
 <tr>
  <td style="width:33%; text-align:center; vertical-align:bottom; font-weight:bold;"><a href="nuttx/"><img style="margin-left:auto; margin-right:auto; padding-bottom:5px;" width="125" height="125" src="https://upload.wikimedia.org/wikipedia/commons/b/b0/NuttX_logo.png"><br/>NuttX</a></td>
  <td style="width:33%; text-align:center; vertical-align:bottom; font-weight:bold;"><a href="freertos/"><img style="margin-left:auto; margin-right:auto; padding-bottom:5px;" width="263" height="100" src="https://upload.wikimedia.org/wikipedia/commons/4/4e/Logo_freeRTOS.png"><br/>FreeRTOS</a></td>
  <td style="width:33%; text-align:center; vertical-align:bottom; font-weight:bold;"><a href="zephyr/"><img style="margin-left:auto; margin-right:auto; padding-bottom:5px;" width="220" height="114" src="/img/posts/logo-zephyr.jpg"><br/>Zephyr</a></td>
 </tr>
</table>

{% include logos_disclaimer.md %}
