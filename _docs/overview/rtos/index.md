---
title: Supported RTOSes
permalink: /docs/overview/rtos/
---

<style>
.rtoscontainer {
  height: 300px;
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
    max-width: 80%;
}
</style>

micro-ROS aims to **bring ROS 2 to microcontrollers** to allow having first-class ROS 2 entities in the embedded world.

The standard approach to micro-ROS assumes a Real-Time Operating System underneath.

Even though recent developments aim at loosening this requirement, with the integration into Arduino IDE as an important step towards true micro-ROS bare-metal support, the RTOS-based support remains the main entrypoint to micro-ROS.

To date, micro-ROS is supported by the RTOSes FreeRTOS, Zephyr, NuttX, in addition to Linux and Windows. 
All three RTOSes are downloaded natively with the [micro-ROS build system](https://github.com/micro-ROS/micro_ros_setup), and can be chosen when creating
a new firmware workspace.
Dedicated tutorials for running your first micro-ROS application on each of these Operating Systems can be found [here](https://micro-ros.github.io/docs/tutorials/core/first_application_rtos/).
The features common to all supported RTOSes are an API compliant with POSIX to some degree, extremely low to low footprint, and availability of different scheduling algorithms to ensure determinism in micro-ROS apps behavior.
Find more details about each of the supported RTOSes below, and a more comprehensive explaination on "Why an RTOS?" in the Concepts section, as this page is meant to provide a schematic overview on the matter.


## Real-Time Operating Systems officially supported by the project

<div class="rtoscontainer">
  <div class="rtositem_description">
    <h3><b>FreeRTOS</b></h3>
    <div>
        <b>Key features:</b>
        <ul>
            <li>Extremely small footprint.</li>
            <li>POSIX extension available.</li>
            <li>Memory management tools</li>
            <li>Standard and idle tasks available with assignable priorities.</li>
            <li>Transport resources: TCP/IP and lwIP.</li>
        </ul>  
        <b>Resources:</b>
        <ul>
            <li><a href="https://www.freertos.org/blog.html">Official FreeRTOS website</a></li>
            <li><a href="https://www.freertos.org/2020/09/micro-ros-on-freertos.html">micro-ROS on FreeRTOS</a></li>
            <li><a href="https://micro-ros.github.io/docs/tutorials/core/first_application_rtos/freertos/">First micro-ROS Application on FreeRTOS</a></li>
        </ul>    
    </div>
  </div>

  <div class="rtositem_image">
    <img src="https://upload.wikimedia.org/wikipedia/commons/4/4e/Logo_freeRTOS.png">
  </div>
</div>

<div class="rtoscontainer">
  <div class="rtositem_description">
    <h3><b>Zephyr</b></h3>
    <div>
        <b>Key features:</b>
        <ul>
            <li>Small footprint.</li>
            <li>Native POSIX port.</li>
            <li>Cross Architecture: Huge collection of <a href="https://docs.zephyrproject.org/latest/boards/index.html">supported boards</a>.</li>
            <li>Extensive suite of Kernel services.</li>
            <li>Multiple Scheduling Algorithms.</li>
            <li>Highly configurable/Modular for flexibility.</li>
            <li>Native Linux, macOS, and Windows Development.</li>
        </ul>  
        <b>Resources:</b>
        <ul>
            <li><a href="https://www.zephyrproject.org/">Official Zephyr website</a></li>
            <li><a href="https://www.zephyrproject.org/micro-ros-a-member-of-the-zephyr-project-and-integrated-into-the-zephyr-build-system-as-a-module/">micro-ROS on Zephyr</a></li>
            <li><a href="https://micro-ros.github.io/docs/tutorials/core/first_application_rtos/zephyr/">First micro-ROS Application on Zephyr</a></li>
            <li><a href="https://micro-ros.github.io/docs/tutorials/advanced/zephyr_emulator/">First micro-ROS Application on Zephyr Emulator</a></li>
        </ul>
    </div>
  </div>

  <div class="rtositem_image">
    <img src="https://upload.wikimedia.org/wikipedia/commons/2/2d/Zephyr-logo.png">
  </div>
</div>

<div class="rtoscontainer">
  <div class="rtositem_description">
    <h3><b>NuttX</b></h3>
    <div>
        <b>Key features:</b>
        <ul>
            <li>POSIX compliant interface to a high degree.</li>
            <li>Rich Feature OS Set</li>
            <li>Highly scalable</li>
            <li>Real-Time behavior: fully pre-emptible; fixed priority, round-robin, and “sporadic” scheduling.</li>
        </ul>  
        <b>Resources:</b>
        <ul>
            <li><a href="https://nuttx.apache.org/">Official NuttX website</a></li>
            <li><a href="https://micro-ros.github.io/docs/tutorials/core/first_application_rtos/nuttx/">First micro-ROS Application on NuttX</a></li>
        </ul>
    </div>
  </div>

  <div class="rtositem_image">
    <img src="https://upload.wikimedia.org/wikipedia/commons/b/b0/NuttX_logo.png">
  </div>
</div>

## Bare metal support

Based on the release of micro-ROS as a standalone library + header files and on the support provided to the Arduino IDE, micro-ROS is available as a bare-metal application, too.
Fin more details in the dedicated [repo](https://github.com/micro-ROS/micro_ros_arduino).

Arduino is an open-source electronics platform based on easy-to-use hardware and software. Arduino boards are able to read inputs and turn it into an output, by sending instructions to the microcontroller on the board. To do so you use the Arduino programming language (based on Wiring), and the Arduino Software (IDE), based on Processing.

<div class="rtoscontainer">
  <div class="rtositem_description">
    <h3><b>Arduino bare-metal support</b></h3>
    <div>
        <b>Key features:</b>
        <ul>
            <li>Inexpensive.</li>
            <li>Cross-platform.</li>
            <li>Simple, clear programming environment.</li>
            <li>Open source and extensible software.</li>
            <li>Open source and extensible hardware.</li>
        </ul>
        <b>Resources:</b>
        <ul>
            <li><a href="https://www.arduino.cc/">Official Arduino Website</a></li>
            <li><a href="https://github.com/micro-ROS/micro_ros_arduino">micro_ros_arduino repo</a></li>
        </ul>    
    </div>
  </div>

  <div class="rtositem_image">
    <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/8/87/Arduino_Logo.svg/720px-Arduino_Logo.svg.png">
  </div>
</div>

