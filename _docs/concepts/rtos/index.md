---
title: Why a Real-Time Operating System?
permalink: /docs/concepts/rtos/
---

The use of Real-Time Operating Systems (RTOS) is a general practice in nowadays embedded systems. These embedded devices typically consist of a resource-constrained microcontroller that executes an application where the interaction with external components is performed. In many cases, this application contains a time-critical task where a time-deadline or deterministic response is required.

Bare-metal applications are also used nowadays, but it requires a very low-level programming skills and lacks of hardware abstraction layers that RTOSes offers. On the other hand, RTOSes typically uses hardware abstraction layers (HAL) that eases the use of hardware resources, such us timers and communication buses, making easier the development and allowing the reuse of code. In addition, they offer thread and tasks entities that, together with the use of schedulers, provides the necessary tools to implement determinism in the applications. The scheduling normally consists of different algorithms where the user can choose from. Another feature that RTOSes normally offers is the stack management, helping in the correct memory usage of the MCU, a valuable resource in embedded-systems.

## RTOS in micro-ROS

Due to the benefits explained in the introduction, micro-ROS integrates RTOS in its software stack. The use of such a tool enhances the micro-ROS features and allows reusing all the tools and implementations they provide. As the micro-ROS software stack is modular, the exchange of software entities is expected and desired. Same happens with the RTOS. Even that NuttX is the *default* RTOS for the project, it can be replaced with Zephyr and FreeRTOS.

As the Operating Systems (OS) that are available for computers, the RTOSes also have different support for standard interfaces. This is established in a family of standards named [POSIX](https://pubs.opengroup.org/onlinepubs/9699919799/). As we aim to port or reuse code of ROS 2 that was natively coded in Linux (a mostly POSIX-compliant OS), the use of RTOSes that complies with these standards is beneficial, as the porting effort of the code is minimal. Same as Linux, NuttX and Zephyr complies at a good degree with POSIX standards, making the porting effort minimal.

Notice that the RTOS call are made by several top layers in the micro-ROS stack. The main one using the RTOS primitives is the middleware. The middleware requires accessing to the transport resources of the RTOS (serial, UDP or 6LoWPAN communications for example), it also requires of the time resources of the RTOS to operate properly. In addition, it is expected that the micro-ROS client library could have access to RTOS resources to have control of mechanisms such as scheduling or power management, so the developer could optimize the application in many domains.

By now, micro-ROS supports three RTOSes, which all come with (basic) POSIX implementations:

* [FreeRTOS](FreeRTOS/)
* [NuttX](NuttX/)
* [Zephyr](Zephyr/)

Most important, we [integrated these RTOSes with the ROS meta build system colcon](integration_with_colcon/).