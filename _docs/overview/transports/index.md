---
title: Transports and Data Links
permalink: /docs/overview/transports/
---

micro-ROS uses the resource-optimized [DDS for Extremely Resource Constrained Environments (DDS-XRCE) standard](https://www.omg.org/spec/DDS-XRCE/), implemented by [eProsima's Micro-XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS/).

It supports are variety of data link and transport protocols. However, the support depends on the underlying RTOS, as specified in the following table.

|                                              | Serial | UDP over Ethernet | UDP over IEEE 802.11 | 6LoWPAN | Bluetooth |
|----------------------------------------------| :----: | :---------------: | :------------------: | :-----: | :-------: |
| [**FreeRTOS**](https://www.freertos.org/)    |        |                   |                      |         |     ✓     |
| [**NuttX**](http://nuttx.apache.org/)        |   ✓    |        ✓          |           ✓          |    ✓    |           |
| [**Zephyr**](https://www.zephyrproject.org/) |   ✓    |                   |                      |         |           |