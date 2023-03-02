---
title: Why a Real-Time Operating System?
permalink: /docs/concepts/rtos/
---

The use of Real-Time Operating Systems (RTOSes) is a general practice in nowadays embedded systems. These systems typically consist of a resource-constrained microcontroller that executes an application which requires an interaction with external components. In many cases, this application contains a time-critical task where a strict time deadline or deterministic response is required.

Bare-metal applications are also used nowadays, but require very low-level programming skills and lack of the hardware abstraction layers that RTOSes offers. On the other hand, RTOSes typically use hardware abstraction layers (HAL) that ease the use of hardware resources, such as timers and communication buses, lightening the development and allowing the reuse of code. In addition, they offer threads and tasks entities which, together with the use of schedulers, provide the necessary tools to implement determinism in the applications. The scheduling consists of different algorithms, among which users can choose the ones that better fits their applications. Another feature that RTOSes normally offer is the stack management, helping in the correct memory usage of the MCU resources, a valuable good in embedded systems.

## RTOS in micro-ROS

Due to the benefits presented above, micro-ROS integrates RTOSes in its software stack. This enhances the capabilities of micro-ROS and allows reusing all the tools and functions provided by the RTOSes. As the micro-ROS software stack is modular, the exchange of software entities is expected and desired at all levels, including the RTOS layer.

Like the Operating Systems (OSes) available for computers, the RTOSes also have different support for standard interfaces. This is established in a family of standards named [POSIX](https://pubs.opengroup.org/onlinepubs/9699919799/). As we aim to port or reuse code of ROS 2 that was natively coded in Linux (a mostly POSIX-compliant OS), the use of RTOSes that comply with these standards is beneficial, as the porting effort of the code is minimal. Both NuttX and Zephyr comply to a good degree with POSIX standards, making the porting effort minimal, whereas FreeRTOS provides a plugin, *FreeRTOS+POSIX*, thanks to which an existing POSIX compliant application can be easily ported to FreeRTOS ecosystem, and therefore leverage all its functionality.

Notice that calls to the RTOS functions are made by several abstraction layers in the micro-ROS stack. The main layer using the RTOS primitives is the middleware. Indeed, it requires accessing the transport resources (serial, UDP or 6LoWPAN communications for example) and the time resources of the RTOS in order to operate properly. In addition, it is desirable that the micro-ROS client libraries (rcl, rclc) have also access to RTOS resources in order to handle mechanisms such as scheduling or power management. In this way, the developer can optimize the application at various levels.

At present, micro-ROS supports three RTOSes, which all come with (basic) POSIX implementations: FreeRTOS, Zephyr and NuttX, all of them [integrated into the micro-ROS build system](/docs/concepts/build_system/).
By clicking on the logos below, you'll be redirected to the Overview section, where the most relevant aspects and key features of each RTOS are presented.

<table style="border:none;">
 <tr>
  <td style="width:33%; text-align:center; vertical-align:bottom; font-weight:bold;"><a href="/docs/overview/rtos/#freertos"><img style="margin-left:auto; margin-right:auto; padding-bottom:5px;" width="263" height="100" src="https://upload.wikimedia.org/wikipedia/commons/4/4e/Logo_freeRTOS.png"><br/>FreeRTOS</a></td>
  <td style="width:33%; text-align:center; vertical-align:bottom; font-weight:bold;"><a href="/docs/overview/rtos/#zephyr"><img style="margin-left:auto; margin-right:auto; padding-bottom:5px;" width="220" height="114" src="/img/posts/logo-zephyr.jpg"><br/>Zephyr</a></td>
  <td style="width:33%; text-align:center; vertical-align:bottom; font-weight:bold;"><a href="/docs/overview/rtos/#nuttx"><img style="margin-left:auto; margin-right:auto; padding-bottom:5px;" width="125" height="125" src="https://upload.wikimedia.org/wikipedia/commons/b/b0/NuttX_logo.png"><br/>NuttX</a></td>
 </tr>
</table>

A thorough technical comparison between these RTOSes can be found [here](/docs/concepts/rtos/comparison/).

{% include logos_disclaimer.md %}
