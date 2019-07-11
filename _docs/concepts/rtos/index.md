---
title: Real-Time Operating Systems (RTOS)
permalink: /docs/concepts/rtos/
redirect_from: /docs/concepts/
---

The use of Real-Time Operating Systems (RTOS) is a general practice in nowadays embedded systems. These embedded devices typically consist of a resource-constrained microcontroller that executes an application where the interaction with external components is performed. In many cases, this application contains a time-critical task where a time-deadline or deterministic response is required.

Bare-metal applications are also used nowadays, but it requires a very low-level programming skills and lacks of hardware abstraction layers that RTOSes offers. On the other hand, RTOSes typically uses hardware abstraction layers (HAL) that eases the use of hardware resources, such us timers and communication buses, making easier the development and allowing the reuse of code. In addition, they offer thread and tasks entities that, together with the use of schedulers, provides the necessary tools to implement determinism in the applications. The scheduling normally consists of different algorithms where the user can choose from. Another feature that RTOSes normally offers is the stack management, helping in the correct memory usage of the MCU, a valuable resource in embedded-systems.

## RTOS in micro-ROS

Due to the benefits explained in the introduction, micro-ROS integrates RTOS in its software stack. The use of such a tool enhances the micro-ROS features and allows reusing all the tools and implementations they provide. As the micro-ROS software stack is modular, the exchange of software entities is expected and desired. Same happens with the RTOS. Even that NuttX is the *default* RTOS for the project, it is expected that several of them could replace it.

As the Operating Systems (OS) that are available for computers, the RTOSes also have different support for standard interfaces. This is established in a family of standards named [POSIX](https://pubs.opengroup.org/onlinepubs/9699919799/). As we aim to port or reuse code of ROS 2 that was natively coded in Linux (a mostly POSIX-compliant OS), the use of RTOSes that complies with these standards is beneficial, as the porting effort of the code is minimal. Same as Linux, NuttX complies at a good degree with POSIX standards, making the porting effort minimal.

Notice that the RTOS call are made by several top layers in the micro-ROS stack. The main one using the RTOS primitives is the middleware. The middleware requires accessing to the transport resources of the RTOS (serial, UDP or 6LOWPAN communications for example), it also requires of the time resources of the RTOS to operate properly. In addition, it is expected that the micro-ROS client library could have access to RTOS resources to have control of mechanisms such as scheduling or power management, so the developer could optimize the application in many domains.

Unfortunately, these resources normally are not POSIX compliant, so the use of abstraction layers at RTOS level is required. These layers will allow to abstract the RTOSes in use and provide unified calls to control aforementioned resources.

## NuttX RTOS

<br/><br/>

<img align="left" width="125" height="125" src="https://upload.wikimedia.org/wikipedia/commons/b/b0/NuttX_logo.png">

[NuttX](http://www.nuttx.org/) is a RTOS that emphasizes its compliance with standards (such us POSIX) and small footprint, it can be fit in 8 to 32 bit microcontrollers. The use of POSIX and ANSI standards, together with the mimic it does to UNIX APIs, makes it friendly to the developers that are used to Linux. The RTOS is licensed under BSD license and makes use of GNU toolchain. In order to obtain more information, please visit [NuttX overview page](http://nuttx.org/Documentation/NuttX.html#overview).

<br/><br/>

## Supported development boards

Check [supported board](/docs/overview/hardware/) section for more information about the boards we are currently using.

## Getting started with NuttX and micro-ROS

In order to obtain more information about how to get started using this RTOS, please check our [tutorials section](/docs/tutorials/basic/getting_started/).
