---
title: Real-Time Operating Systems (RTOS)
permalink: /docs/concepts/rtos/
---

# RTOS

## Introduction

The use of Real-Time Operating Systems (RTOS) is a general practice in nowadays embedded systems. These embedded devices typically consist of a resource-constrained microcontroller that executes an application where the interaction with external components is performed. In many cases, this application contains a time-critical task where a time-deadline or deterministic response is required.

Bare-metal applications are also used nowadays, but it requires a very low-level programming skills and lacks of hardware abstraction layers that RTOSes offers. On the other hand, RTOSes typically uses hardware abstraction layers (HAL) that eases the use of hardware resources, such us timers and communication buses, making easier the development and allowing the reuse of code. In addition, they offer thread and tasks entities that, together with the use of schedulers, provides the necessary tools to implement determinism in the applications. The scheduling normally consists of different algorithms where the user can choose from. Another feature that RTOSes normally offers is the stack management, helping in the correct memory usage of the MCU, a valuable resource in embedded-systems.

## RTOS in micro-ROS

Due to the benefits explained in the introduction, micro-ROS integrates RTOS in its software stack. The use of such a tool enhances the micro-ROS features and allows reusing all the tools and implementations they provide. As the micro-ROS software stack is modular, the exchange of software entities is expected and desired. Same happens with the RTOS. Even that NuttX is the *default* RTOS for the project, it is expected that several of them could replace it.

## NuttX RTOS

[NuttX](http://www.nuttx.org/) is a RTOS that emphasizes its compliance with standards (such us POSIX) and small footprint, it can be fit in 8 to 32 bit microcontrollers. The use of POSIX and ANSI standards, together with the mimic it does to UNIX APIs, makes it friendly to the developers that are used to Linux. The RTOS is licensed under BSD license and makes use of GNU toolchain. In order to obtain more information, please visit [NuttX overview page](http://nuttx.org/Documentation/NuttX.html#overview).

![NuttX_logo](https://upload.wikimedia.org/wikipedia/commons/b/b0/NuttX_logo.png)


## Supported development boards

For development purposes, project consortium has established two development boards as development blueprints: [Olimex STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware) and the [STM32LDiscovery](https://www.st.com/en/evaluation-tools/32l152cdiscovery.html). Both development board schematics are open, promoting new hardware designs based on such a platforms.

The first one consists on a ARM Cortex-M4F MCU with 196 KB of RAM and 1 MB of flash. It also offers Arduino-like expansion pins and Ethernet communication means.

The STM32LDiscovery board contains a STM32L ultra-low power packaging and consists of a ARM Cortex-M3 MCU that integrates 32KB of RAM, 256KB of flash memory. This microcontroller aims to target low-power applications.

## Getting started with NuttX and micro-ROS

In order to obtain more information about how to get started using this RTOS, please check our [documentation repository](https://github.com/microROS/micro-ROS-doc), where tutorials and getting started material is offered.

We have created several Docker containers, where some are meant for development purposes and other to execute precompiled examples. These Docker files have been gathered together in the [micro-ROS Docker repository](https://github.com/microROS/docker).
