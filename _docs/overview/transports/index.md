---
title: Transports and Data Links
permalink: /docs/overview/transports/
---

micro-ROS uses the resource-optimized [DDS for Extremely Resource Constrained Environments (DDS-XRCE) standard](https://www.omg.org/spec/DDS-XRCE/), implemented by [eProsima's Micro-XRCE-DDS](https://github.com/eProsima/Micro-XRCE-DDS/).

It supports are variety of data link and transport protocols. However, the support depends on the underlying RTOS and the selected hardware.

The following table specifies the available transports for the **officially supported** boards and RTOS. You can find detailed information about hardware support [here](/docs/overview/hardware/).

|                    | [**NuttX**](http://nuttx.apache.org/) | [**FreeRTOS**](https://www.freertos.org/) | [**Zephyr**](https://www.zephyrproject.org/) | [Arduino](https://github.com/micro-ROS/micro_ros_arduino) |
| ------------------ | :-----------------------------------: | :---------------------------------------: | :------------------------------------------: | :---------------------------------------------------------------------: |
| Olimex STM32-E407  |          USB, UART, Network           |               UART, Network               |                  USB, UART                   |                                    -                                    |
| ST B-L475E-IOT01A  |                   -                   |                     -                     |              USB, UART, Network              |                                    -                                    |
| Crazyflie 2.1      |                   -                   |             Custom Radio Link             |                      -                       |                                    -                                    |
| Espressif ESP32    |                   -                   |              UART, WiFI UDP               |                      -                       |                                    -                                    |
| Teensy 3.2         |                   -                   |                     -                     |                      -                       |                                USB, UART                                |
| Teensy 4.0/4.1     |                   -                   |                     -                     |                      -                       |                                USB, UART                                |
| ROBOTIS OpenCR 1.0 |                   -                   |                     -                     |                      -                       |                                USB, UART                                |

Regarding the **community supported** boards, at the moment of writing the available transports are:

|                  | [**NuttX**](http://nuttx.apache.org/) | [**FreeRTOS**](https://www.freertos.org/) | [**Zephyr**](https://www.zephyrproject.org/) | [Arduino](https://github.com/micro-ROS/micro_ros_arduino) |
| ---------------- | :-----------------------------------: | :---------------------------------------: | :------------------------------------------: | :---------------------------------------------------------------------: |
| Arduino Due      |                   -                   |                     -                     |                      -                       |                                USB, UART                                |
| Arduino Zero     |                   -                   |                     -                     |                      -                       |                                USB, UART                                |
| ST Nucleo F446ZE |                   -                   |                   UART                    |                      -                       |                                    -                                    |
| ST Nucleo H743ZI |                   -                   |                   UART                    |                      -                       |                                    -                                    |
| ST Nucleo F746ZG |                   -                   |                   UART                    |                      -                       |                                    -                                    |

