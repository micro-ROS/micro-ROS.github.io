---
title: Overview
permalink: /docs/tutorials/demos/overview/
redirect_from:
  - /docs/tutorials/demos/
---

In addition to the above tutorials, we created a collection of demos that showcase micro-ROS in real applications which are easy to reproduce by the community &mdash; and thus _you_. Unlike tutorials, we do not explain the demo code step by step, but provide ready-to-use Docker files to get you started as quickly as possible.

Choose yourself:

|              | [**Kobuki Demo**](../kobuki_demo/) | [**Crazyflie Demo**](../crazyflie_demo/) | [**ToF Sensor Demo**](../tof_demo/) |
| --- | :-: | :-: | :-: |
| _RTOS_       | NuttX                             | FreeRTOS                                | Zephyr                             |
| _Hardware\*_ | Kobuki, Olimex LTD STM32-E407     | Crazyflie, radio, flow deck             | ST B-L475E-IOT01A Discovery kit    |
| _HW Costs\*_ | ≈ 360$                            | ≈ 330$                                  | ≈ 60$                              |
|              | <img src="kobuki.png" style="margin:auto;"/> | <img src="crazyflie.png" style="margin:auto;"/> |                 |

<div style="font-size:80%;color:gray;text-align:right;margin-bottom:1em;">*As a matter of course, you'll need a computer/laptop, various cables, and<br/>further auxiliary equipment. The hardware costs are only a rough estimate.</div>

The demos may also interface with the FIWARE Context Broker, which is the core of the [FIWARE](https://www.fiware.org/) open source initiative for context data management. Learn more at [**Interfacing with FIWARE Context Broker**](../fiware_demo/).

Finally, the demos can be combined in several ways to demonstrate further functionalities. Turn to [**Combined Demos**](../combined_demos/) to learn more about these combinations and how to launch them.

