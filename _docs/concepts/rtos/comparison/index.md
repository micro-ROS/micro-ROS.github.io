---
title: Comparison between RTOSes
permalink: /docs/concepts/rtos/comparison/
---

In this section, we present a techincal comparison between the three RTOSes supported by micro-ROS:
* [NuttX](https://nuttx.apache.org/)
* [FreeRTOS](https://www.freertos.org/)
* [Zephyr](https://www.zephyrproject.org/)

The comparison regards the features listed below:
* Standardized API to application level
* Maturity
* Supported hardware
* Scheduling options
* IO Support (native or vendor-specific module needed)
* Networking stack
* Storage and Display
* Memory Footprint
* Safety Certification
* License
* POSIX level support

**Key questions:**
* Evaluation POSIX-compliance of RTOS.
* What is the effort in providing an additional layer for non-POSIX RTOS regarding micro-ROS or ROS 2?
* Support of RTOS for specific HW platforms

## Table of Comparisons

| **OS**                                                       | **NuttX**                                | **FreeRTOS**                                                                                                             | **Zephyr**                                                                                      |
| ------------------------------------------------------------ | ---------------------------------------- | ------------------------------------------------------------------------------------------------------------------------ | ----------------------------------------------------------------------------------------------- |
| **Feature**                                                  |                                          |                                                                                                                          |                                                                                                 |
| **Standardization**                                          |                                          |                                                                                                                          |                                                                                                 |
| POSIX                                                        | yes                                      | partial                                                                                                                  | partial                                                                                         |
| POSIX.1 <sup>1</sup>                                         | [yes](http://nuttx.org/)                 | [wrapper](https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_POSIX/index.html)                                         | partial                                                                                         |
| POSIX.1b <sup>2</sup>                                        | yes                                      | partial                                                                                                                  | partial                                                                                         |
| POSIX.1c <sup>3</sup>                                        | yes                                      | yes                                                                                                                      | partial                                                                                         |
|                                                              |                                          |                                                                                                                          |                                                                                                 |
| OSEK/VDX                                                     | no                                       | no                                                                                                                       | no                                                                                              |
| **Maturity**                                                 |                                          |                                                                                                                          |                                                                                                 |
| First release                                                | 2007                                     | 2003                                                                                                                     | 2016                                                                                            |
| Last release                                                 | 2019                                     | 2020                                                                                                                     | 2019                                                                                            |
| Update rate                                                  | about 3 months                           | irregular                                                                                                                | 3 months                                                                                        |
| Community                                                    | open-source                              | open-source                                                                                                              | Linux Foundation Collaboration Project, (Intel, Linaro (ARM), nordic, NXP, Synopsys)            |
|                                                              |                                          |                                                                                                                          |                                                                                                 |
| **Supported Hardware**                                       |                                          |                                                                                                                          |                                                                                                 |
| Olimex STM32-E407 (Cortex-M4)                                | yes                                      | yes                                                                                                                      | yes, [explicitly](https://docs.zephyrproject.org/latest/kernel/services/scheduling/index.html) |
| Bosch XDK <sup>5</sup>                                       | not explicitly, but similar <sup>6</sup> | yes                                                                                                                      | no<sup>5</sup>                                                                                  |
| MPC57xx                                                      | no                                       | no                                                                                                                       | no                                                                                              |
| **Scheduling**                                               |                                          |                                                                                                                          |                                                                                                 |
| Priority-based                                               | FIFO                                     | yes                                                                                                                      | yes                                                                                             |
| Round-Robin <sup>4</sup>                                     | yes                                      | yes <sup>6</sup>                                                                                                         | [co-operative](https://docs.zephyrproject.org/latest/kernel/services/scheduling/index.html)    |
| Sporadic Server                                              | yes                                      | no                                                                                                                       | no                                                                                              |
| Reservation Based Scheduling (RBS)                           | no                                       | no                                                                                                                       | no                                                                                              |
| Rate Monotonic Scheduling (RMS)                              | ?                                        | yes <sup>10</sup>                                                                                                        | yes <sup>10</sup>                                                                               |
| Semaphore /Mutex Management                                  | yes (Priority Inheritance)               | yes                                                                                                                      | yes                                                                                             |
| **IO**                                                       |                                          |                                                                                                                          |                                                                                                 |
| I2C                                                          | yes                                      | yes <sup>8</sup>                                                                                                         | yes                                                                                             |
| SPI                                                          | yes                                      | yes <sup>8</sup>                                                                                                         | yes                                                                                             |
| UART                                                         | hw-specific                              | yes <sup>8</sup>                                                                                                         | yes                                                                                             |
| USB                                                          | yes                                      | vendor-specific                                                                                                          | yes                                                                                             |
| CAN                                                          | yes                                      | vendor-specific                                                                                                          | yes                                                                                             |
| CAnopen                                                      | no                                       | vendor-specific                                                                                                          | yes                                                                                             |
| Modbus                                                       | yes                                      | vendor-specific                                                                                                          | ?                                                                                               |
| **Networking** <sup>7</sup>                                  |                                          |                                                                                                                          |                                                                                                 |
| BLE-Stack                                                    | unclear                                  | yes <sup>8</sup>                                                                                                         | yes                                                                                             |
| 6LoWPAN                                                      | yes                                      | no                                                                                                                       | yes                                                                                             |
| TLS                                                          |                                          | yes <sup>8</sup>                                                                                                         | yes                                                                                             |
| Thread                                                       |                                          | no                                                                                                                       | ?                                                                                               |
| Ethernet                                                     | yes                                      | yes <sup>8</sup>                                                                                                         | yes                                                                                             |
| Wifi                                                         | yes                                      | yes <sup>8</sup>                                                                                                         | yes                                                                                             |
| NFC                                                          | unclear                                  | no                                                                                                                       | yes                                                                                             |
| RFID                                                         | yes                                      | no                                                                                                                       | yes                                                                                             |
| **Storage & Display** <sup>7</sup>                           |                                          |                                                                                                                          |                                                                                                 |
| File System                                                  | yes                                      | yes                                                                                                                      | yes                                                                                             |
| Graphical User Interface                                     |                                          | vendor-specific                                                                                                          | ?                                                                                               |
| **Memory Footprint**                                         |                                          |                                                                                                                          |                                                                                                 |
| RAM                                                          | "small footprint"                        | 236 B scheduler + 64 B / task                                                                                            | "small footprint"                                                                               |
| ROM                                                          | "small footprint"                        | 5 - 10 kB                                                                                                                | "small footprint"                                                                               |
| **Safety Certification**                                     |                                          |                                                                                                                          |                                                                                                 |
| Software Development Process DO178B Level A / EUROCAE ED-12B | no                                       | [SafeRTOS: DO178C (Aerspace) by Wittenstein](https://www.highintegritysystems.com/safertos/certification-and-standards/) | no                                                                                              |
| Functional Safety IEC-61508                                  | no                                       | [SafeRTOS (SIL 3)](http://www.openrtos.net/FreeRTOS-Plus/Safety_Critical_Certified/SafeRTOS-Safety-Critical-Certification.shtml)                      | soon                                                                                            |
| **License**                                                  | BSD                                      | MIT and Commercial                                                                                                       | Apache 2                                                                                        |

<sup>1</sup> Processes, signals, fpe, segmentation, bus errors, timers, file and directory ops, pipes, c library, IO Port Interface

<sup>2</sup> Real-time, clocks, semaphores, messages, shared mem, async io, memory locking.

<sup>3</sup> Threads.

<sup>4</sup> Executing every task in round-robin fashion but only for a pre-defined time slice.

<sup>5</sup> XDK is based on a microcontroller of the ARM Cortex M3 EFM32GG390F1024 Giant Gecko family by Silicon Labs. Note that the version used in the XDK is not fully supported by Zephyr.

<sup>6</sup> [Note: Time slicing](https://www.freertos.org/media/2018/161204_Mastering_the_FreeRTOS_Real_Time_Kernel-A_Hands-On_Tutorial_Guide.pdf)

<sup>7</sup> Hardware-support for Networking and Storage often depends on the platform and sometimes packages of hardware-vendors are available, which work for a particular operating system. But it is in general difficult to determine the harware-support of a given RTOS.

<sup>8</sup> FreeRTOS interfaces that are provided through the
FreeRTOS reference distribution repository at https://github.com/aws/amazon-freertos

<sup>9</sup> FreeRTOS ethernet support is provided through the FreeRTOS+TCP stack.

<sup>10</sup> Rate Monotonic Scheduling can be achieved by assigning the priorities of threads with the so-called rate-monotonic policy. That is, the thread with the highest rate has the highest priority and the thread with the lowest rate the lowest priority. It is assumed that all threads are activated periodically with fixed rates.

Some Related Work:
* [Choosing the right RTOS for IoT platform, Milinkovic et al, INFOTEH-JAHORINA Vol. 14, 2015](http://infoteh.rs.ba/zbornik/2015/radovi/RSS-2/RSS-2-2.pdf): comparison of FreeRTOS, ChibiOS/RT, Erika, RIOT
* [FreeRTOS Architecture](https://www.freertos.org/)
