---
title: First micro-ROS Application on an RTOS
permalink: /docs/tutorials/core/first_application_rtos/
---

After you have completed the [First micro-ROS application on Linux tutorial](../first_application_linux), you are know ready to flash a microcontroller with this application based on a Real-Time Operating System (RTOS).

Micro-ROS currently supports three different RTOS, namely NuttX, FreeRTOS, and Zephyr. Of course, the micro-ROS-related sections of the application code are independent of the underlying RTOS. Also, the basic tooling is the same as we have integrated the RTOS tools with the ROS 2 meta build system colcon. However, there are subtle differences in the configuration and the definition of the executables between the three RTOS. Therefore, for this tutorial, please decide for one RTOS to use:

* [NuttX](nuttx/)
* [FreeRTOS](freertos/)
* [Zephyr](zephyr/)
