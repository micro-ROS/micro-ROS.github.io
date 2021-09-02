---
title: Supported RTOSes
permalink: /docs/overview/rtos/
---

<style>
.rtoscontainer {
  height: auto;
  display: flex;
  flex-direction: row;
  justify-content: flex-start;
  flex-wrap: wrap;
}

.rtositem_image {
  width: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
}

.rtositem_description {
  width: 50%;
}

.rtositem_image img {
    max-width: 70%;
}
</style>

micro-ROS aims to **bring ROS 2 to microcontrollers** to allow having first-class ROS 2 entities in the embedded world.

The standard approach to micro-ROS assumes a Real-Time Operating System underneath.

Even though recent developments aim at loosening this requirement, with the integration into Arduino IDE as an important step towards true micro-ROS bare-metal support, the RTOS-based support remains the main entrypoint to micro-ROS.

To date, micro-ROS is supported by the RTOSes FreeRTOS, Zephyr, NuttX, in addition to Linux and Windows.
All three RTOSes are downloaded natively with the [micro-ROS build system](https://github.com/micro-ROS/micro_ros_setup), and can be chosen when creating
a new firmware workspace.
Dedicated tutorials for running your first micro-ROS application on each of these Operating Systems can be found [here](https://micro-ros.github.io/docs/tutorials/core/first_application_rtos/).
The features common to all supported RTOSes are an API compliant with POSIX to some degree, extremely low-to-low memory footprint, and availability of different scheduling algorithms to ensure determinism in micro-ROS apps behavior.

Find more details about each of the supported RTOSes below.
For a more comprehensive explanation regarding the choice of working with Real-Time Operating Systems, and for a more technical comparison among these three RTOSes, please refer to the [RTOS page in the Concepts section](https://micro-ros.github.io/docs/concepts/rtos/), as the present page is meant to solely provide a schematic overview.


## Real-Time Operating Systems officially supported by the project

In this section, we review the main features of the three RTOSes supported officially by the project, and provide links to useful documentation.

### **FreeRTOS**

FreeRTOS is distributed under the MIT license. It is known particularly for its simplicity and the extension a:FreeRTOS provided by Amazon. For micro-ROS, we make use of the POSIX extension.

<div class="rtoscontainer">
  <div class="rtositem_description">
    <div>
        <b>Key features:</b>
        <ul>
            <li>Extremely small footprint</li>
            <li>POSIX extension available</li>
            <li>Memory management tools</li>
            <li>Standard and idle tasks available with assignable priorities</li>
            <li>Transport resources: TCP/IP and lwIP</li>
        </ul>
        <b>FreeRTOS resources:</b>
        <ul>
            <li><a href="https://www.freertos.org/">Official FreeRTOS website</a></li>
            <li><a href="https://www.freertos.org/a00104.html">Download</a></li>
            <li><a href="https://www.freertos.org/Documentation/RTOS_book.html">Documentation</a></li>
        </ul>
        <b>FreeRTOS &amp; micro-ROS:</b>
        <ul>
            <li><a href="https://www.freertos.org/2020/09/micro-ros-on-freertos.html">micro-ROS on FreeRTOS</a></li>
            <li><a href="https://micro-ros.github.io/docs/tutorials/core/first_application_rtos/freertos/">First micro-ROS Application on FreeRTOS</a></li>
        </ul>
    </div>
  </div>

  <div class="rtositem_image">
    <img src="https://upload.wikimedia.org/wikipedia/commons/4/4e/Logo_freeRTOS.png">
  </div>
</div>

### **Zephyr**

Zephyr is a fairly new open-source RTOS, developed in a Linux Foundation Project. The members of this project include several renowned semiconductor companies. Zephyr strives for a functional safety certification, which would make it the first open-source RTOS with such a certification.

<div class="rtoscontainer">
  <div class="rtositem_description">
    <div>
        <b>Key features:</b>
        <ul>
            <li>Small footprint</li>
            <li>Native POSIX port</li>
            <li>Cross Architecture: Huge collection of <a href="https://docs.zephyrproject.org/latest/boards/index.html">supported boards</a></li>
            <li>Extensive suite of Kernel services</li>
            <li>Multiple Scheduling Algorithms</li>
            <li>Highly configurable/Modular for flexibility</li>
            <li>Native Linux, macOS, and Windows Development</li>
        </ul>
        <b>Zephyr resources:</b>
        <ul>
            <li><a href="https://www.zephyrproject.org/">Official Zephyr website</a></li>
            <li><a href="https://docs.zephyrproject.org/latest/guides/west/">Meta-tool *West*</a></li>
            <li><a href="https://github.com/zephyrproject-rtos/zephyr">Download</a></li>
            <li><a href="https://docs.zephyrproject.org/latest/">Documentation</a></li>
        </ul>
        <b>Zephyr &amp; micro-ROS:</b>
        <ul>
            <li><a href="https://www.zephyrproject.org/micro-ros-a-member-of-the-zephyr-project-and-integrated-into-the-zephyr-build-system-as-a-module/">micro-ROS on Zephyr</a></li>
            <li><a href="https://micro-ros.github.io/docs/tutorials/core/first_application_rtos/zephyr/">First micro-ROS Application on Zephyr</a></li>
            <li><a href="https://micro-ros.github.io/docs/tutorials/core/zephyr_emulator/">First micro-ROS Application on Zephyr Emulator</a></li>
        </ul>
    </div>
  </div>

  <div class="rtositem_image">
    <img src="https://www.linuxfoundation.org/wp-content/uploads/zephyr-color.svg">
  </div>
</div>

### **NuttX**

NuttX emphasizes its compliance with standards - including POSIX - and small footprint. It can be fit on 8- to 32-bit microcontrollers. The use of POSIX and ANSI standards, together with the mimic it does to UNIX APIs, makes it friendly to the developers that are used to Linux. NuttX is licensed under BSD license and makes use of the GNU toolchain. Please note that the uClib++ library used with NuttX comes under the stricter GNU LGPL Version 3 license.

<div class="rtoscontainer">
  <div class="rtositem_description">
    <div>
        <b>Key features:</b>
        <ul>
            <li>POSIX compliant interface to a high degree</li>
            <li>Rich Feature OS Set</li>
            <li>Highly scalable</li>
            <li>Real-Time behavior: fully pre-emptible; fixed priority, round-robin, and “sporadic” scheduling</li>
        </ul>
        <b>NuttX resources:</b>
        <ul>
            <li><a href="https://nuttx.apache.org/">Official NuttX website</a></li>
            <li><a href="https://nuttx.apache.org/download/">Download</a></li>
            <li><a href="https://cwiki.apache.org/confluence/display/NUTTX/Nuttx">Documentation</a></li>
        </ul>
        <b>NuttX &amp; micro-ROS:</b>
        <ul>
            <li><a href="https://micro-ros.github.io/docs/tutorials/core/first_application_rtos/nuttx/">First micro-ROS Application on NuttX</a></li>
        </ul>
    </div>
  </div>

  <div class="rtositem_image">
    <img src="https://upload.wikimedia.org/wikipedia/commons/b/b0/NuttX_logo.png">
  </div>
</div>

### **RT-Thread**

RT-Thread is an open source, neutral, and community-based real-time operating system (RTOS). RT-Thread has Standard version and Nano version. For resource-constrained microcontroller (MCU) systems, the NANO kernel version that requires only 3 KB Flash and 1.2 KB RAM memory resources can be tailored with easy-to-use tools; And for resource-rich IoT devices, RT-Thread can use the online software package management tool, together with system configuration tools, to achieve intuitive and rapid modular cutting, seamlessly import rich software packages, thus achieving complex functions like Android's graphical interface and touch sliding effects, smart voice interaction effects, and so on.

<div class="rtoscontainer">
  <div class="rtositem_description">
    <div>
        <b>Key features:</b>
        <ul>
            <li>Designed for resource-constrained devices, the minimum kernel requires only 1.2KB of RAM and 3 KB of Flash.</li>
            <li>Has rich components and a prosperous and fast growing <a href="https://packages.rt-thread.org/en/">package ecosystem</a>.</li>
            <li>Elegant code style, easy to use, read and master.</li>
            <li>High Scalability. RT-Thread has high-quality scalable software architecture, loose coupling, modularity, is easy to tailor and expand.</li>
            <li>Supports high-performance applications.</li>
            <li>Supports cross-platform and a wide range of <a href="https://www.rt-thread.io/board.html">chips</a>.</lili>
        </ul>
        <b>RT-Thread resources:</b>
        <ul>
            <li><a href="https://www.rt-thread.io/">Official RT-Thread website</a></li>
            <li><a href="https://github.com/rt-thread/rt-thread">Download</a></li>
            <li><a href="https://www.rt-thread.io/document/site/">Documentation</a></li>
            <li><a href="https://www.rt-thread.io/studio.html">RT-Thread Studio</a></li>
        </ul>
        <b>RT-Thread &amp; micro-ROS:</b>
        <ul>
            <li><a href="https://github.com/wuhanstudio/micro_ros">First micro-ROS Application on RT-Thread</a></li>
        </ul>
    </div>
  </div>


  <div class="rtositem_image">
    <img src="https://www.rt-thread.io/images/logo_down.png">
  </div>
</div>


## Bare metal support

Based on the release of micro-ROS as a standalone library with header files, and on the support provided to the Arduino IDE, micro-ROS is available as a bare-metal application, too.
Find more details in the dedicated [repo](https://github.com/micro-ROS/micro_ros_arduino).

### **Arduino bare-metal support**

The open-source Arduino Software (IDE) is a library making it easy to program any Arduino board.

<div class="rtoscontainer">
  <div class="rtositem_description">
    <div>
        <b>Key features:</b>
        <ul>
            <li>Inexpensive</li>
            <li>Cross-platform</li>
            <li>Simple, clear programming environment</li>
            <li>Open source and extensible software</li>
            <li>Open source and extensible hardware</li>
        </ul>
        <b>Resources:</b>
        <ul>
            <li><a href="https://www.arduino.cc/">Official Arduino Website</a></li>
            <li><a href="https://github.com/micro-ROS/micro_ros_arduino">micro_ros_arduino repo</a></li>
        </ul>
    </div>
  </div>

  <div class="rtositem_image">
    <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/8/87/Arduino_Logo.svg/720px-Arduino_Logo.svg.png" width="500">
  </div>
</div>

## Experimentally supported Real-Time Operating Systems

### **Arm® Mbed™ OS**

Mbed OS is an open-source RTOS intended for IoT applications with 32-bit ARM Cortex-M microcontrollers.

<div class="rtoscontainer">
  <div class="rtositem_description">
    <div>
        <b>Key features:</b>
        <ul>
            <li>Small footprint</li>
            <li>Many POSIX-compatible modules</li>
            <li>Preemptive scheduling</li>
            <li>Support of Arm Compiler and GNU Arm Embedded</li>
            <li>Online compiler at <a href="https://os.mbed.com/compiler/">https://os.mbed.com/compiler/</a></li>
        </ul>
        <b>Mbed OS resources:</b>
        <ul>
            <li><a href="https://os.mbed.com/">Official Mbed website</a></li>
            <li><a href="https://github.com/ARMmbed/mbed-os">Source code</a></li>
            <li><a href="https://os.mbed.com/docs/">Documentation</a></li>
        </ul>
        <b>Mbed OS &amp; micro-ROS:</b>
        <ul>
            <li><a href="https://github.com/micro-ROS/micro_ros_mbed">micro-ROS example for Mbed</a></li>
        </ul>
    </div>
  </div>

  <div class="rtositem_image">
    <!-- Use of logo requires explicit permission, cf. https://www.arm.com/company/policies/trademarks/arm-trademark-list/mbed-trademark.
         This should be considered if the support is no longer experimental only. -->
  </div>
</div>

{% include logos_disclaimer.md %}
