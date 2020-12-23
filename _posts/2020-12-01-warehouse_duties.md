---
title: New demo - micro-ROS enabling smart warehouse duties
author: francesca-finocchiaro
---

In this brand new demo, micro-ROS showcases its capabilities in an environment that mimics a realistic industrial scenario like a smart warehouse, comprising a dynamic and distributed system of heterogeneous IoT devices. The demo was designed, orchestrated and recorded by OFERA partner [PIAP](https://piap.pl/en/).

It presents a mobile platform interacting with a set of sensors and actuators based on microcontrollers running micro-ROS over 6LoWPAN and scattered in a simulated warehouse area, both indoor and outdoor, and is meant to show the seamless integration of these basic elements into a complex ROS 2 ecosystem thanks to micro-ROS.

<iframe width="560" height="315" src="https://www.youtube.com/embed/8L1XVsZMmsI" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>

The mobile base is a *scout* robot designed for quick recognition of the surroundings, thanks to a localisation system providing coherent information on the platform position, and it features an on-board computer running a ROS 2 node. A regular computer is connected to the system via Wi-Fi, with which the ground station operator defines the tasks building up the overall mission, provides the required "world model" to execute it, and interchanges messages with the robot. An autonomy module is then in charge to drive the robot all the way down to the mission accomplishment.

During its trajectory, the robot steps into a series of environmental devices, with which it intercommunicates when they are within the range of the wireless 6LoWPAN protocol, namely:

- A humidity and temperature sensor from which it fetches measurements and sends them to the ground station operator;
- A door which can be opened remotely;
- A ToF (Time of Flight) sensor measuring distances and communicating whether the path is clear;
- A light that can be turned on and off.

The communication is mediated by a micro-ROS Agent running on the on-board computer, which connects to the various micro-ROS Clients operating the sensors and actuators listed above as the robot moves across the simulated warehouse. In all devices, micro-ROS is running on an [Olimex LTD STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware) board, on top of the [NuttX](https://nuttx.apache.org/) RTOS.
