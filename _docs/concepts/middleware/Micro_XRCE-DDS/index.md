---
title: Micro XRCE-DDS
permalink: /docs/concepts/middleware/Micro_XRCE-DDS/
redirect_from: /docs/concepts/middleware/
---

**eProsima Micro XRCE-DDS** is an open-source wire protocol that implements the OMG DDS for e**X**tremely **R**esource **C**onstrained **E**nvironment standard ([DDS-XRCE](https://www.omg.org/spec/DDS-XRCE/About-DDS-XRCE/)).
The aim of the DDS-XRCE protocol is to provide access to the DDS Global-Data-Space from resource-constrained devices.
This is achieved thanks to a **client-server** architecture, where low resource devices, called *XRCE Clients*, are connected to a server, called *XRCE Agent*, which acts on behalf of its clients in the DDS Global-Data-Space.

![](uxrce_scope.png)

Micro XRCE-DDS is composed by two main elements:

* [Micro XRCE-DDS Agent](https://github.com/eProsima/Micro-XRCE-DDS-Agent): a **C++11 out-of-the-box application** which implements the XRCE Agent functionality.
* [Micro XRCE-DDS Client](https://github.com/eProsima/Micro-XRCE-DDS-Client): a **C99 library** which implements the XRCE Client side functionality.

Apart of these, Micro XRCE-DDS uses other two components:

* [Micro CDR](https://github.com/eProsima/Micro-CDR): a **de/serialization engine** used in the Client library.
* [Micro XRCE-DDS Gen](https://github.com/eProsima/Micro-XRCE-DDS-Gen): a **code generator tool** used for generating *Micro CDR* de/serialization function and Client apps examples from IDL sources.

## Application

Micro XRCE-DDS is focused on microcontroller applications which require a publisher/subscriber architecture.
Some examples of this kind of applications are those found in sensor networks, IoT or robotics.
On this last regard, some companies such as [Renesas](https://www.sensorsmag.com/iot-wireless/mcus-support-dds-xrce-protocol-for-ros-2) and [ROBOTIS](https://xelnetwork.readthedocs.io/en/latest/) are using Micro XRCE-DDS as the middleware solution.
Furthermore, the [micro-ROS](https://microros.github.io) project, whose target is to put ROS 2 onto microcontroller, has adopted Micro XRCE-DDS as the middleware layer.

## Main Features

### Low Resource Consumption

As it was aforementioned, Micro XRCE-DDS is focused on microcontroller applications. Therefore, the design and implementation of this middleware have been carried out taking into account the memory restriction of this kind of devices.
Proof of this is the fact that Micro XRCE-DDS Client is completely dynamic memory free.
From the point of view of memory footprint, the [latest](https://github.com/eProsima/Micro-XRCE-DDS-Client/releases/tag/v1.0.1) version of this library has a memory consumption of less than **75 KB of Flash memory** and **2.5 KB of RAM** for a complete publisher and subscriber application.

### Multi-Transport Support

In contrast to other IoT middleware such as MQTT and CoaP which work over only a particular transport layer, XRCE support multiple transport protocol natively.
In particular, the latest version of Micro XRCE-DDS support: **UDP**, **TCP** and a custom **Serial** transport protocol.

Apart from this, Micro XRCE-DDS has a transport interface for both Agent and Client which allows to implement custom transport in an straight-forward manner.
This makes the port of Micro XRCE-DDS to different platforms and the addition of new transports a task that any user can undertake.

### Multi-Platform Support

Micro XRCE-DDS Client support **Windows**, **Linux** and **NuttX** as embedded RTOS.
On the other hand, Micro XRCE-DDS Agent support Windows and Linux platform.

## Other links

* [Manual at Read the Docs](https://micro-xrce-dds.readthedocs.io/en/latest/)
* [GitHub](https://github.com/eProsima/Micro-XRCE-DDS)

