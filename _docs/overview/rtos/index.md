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

Even though recent developments aim at loosening this requirement, with the integration into Arduino IDE being a first step towards true micro-ROS bare-metal support, the RTOS-based support remains the main entrypoint to micro-ROS.

micro-ROS is supported by the RTOSes FreeRTOS, Zephyr, NuttX, in addition to Linux and Windows. The features common to all supported RTOSes are a POSIX compliant API to some degree, extremely low to low footprint, and availability of different scheduling algorithms to ensure determinism in micro-ROS apps behavior.
Find more details about each of the supported RTOSes below, and a more comprehensive explaination on "Why an RTOS?" in the Concepts section, as this page is meant to provide a schematic overview on the matter.


## Real-Time Operating Systems officially supported by the project

The micro-ROS Tier 2 boards are officially supported for one or more RTOSes and transports.

<div class="rtoscontainer">
  <div class="rtositem_description">
    <h3><b>FreeRTOS</b></h3>
    <div>
        <b>Key features:</b>
        <ul>
            <li>Extremely small footprint</li>
            <li>RAM: 520 kB</li>
            <li>Flash: 4 MB</li>
            <li>Peripherals: Ethernet MAC, Wi-Fi 802.11 b/g/n, Bluetooth v4.2 BR/EDR, BLE, SPI, I2C, I2S, UART, SDIO, CAN, GPIO, ADC/DAC, PWM  </li>
        </ul>  
        <b>Resources:</b>
        <ul>
            <li><a href="https://www.freertos.org/blog.html">Official website</a></li>
            <li><a href="https://www.freertos.org/2020/09/micro-ros-on-freertos.html">micro-ROS on FreeRTOS</a></li>
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
            <li>MCU: ARM Cortex-M4 MK20DX256VLH7</li>
            <li>RAM: 64 kB</li>
            <li>Flash: 256 kB</li>
            <li>Peripherals: USB, SPI, I2C, CAN, I2S... </li>
        </ul>  
        <b>Resources:</b>
        <ul>
            <li><a href="https://www.zephyrproject.org/">Official website</a></li>
            <li><a href="https://www.zephyrproject.org/micro-ros-a-member-of-the-zephyr-project-and-integrated-into-the-zephyr-build-system-as-a-module/">micro-ROS on Zephyr</a></li>
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
        <p>NuttX is a real-time operating system (RTOS) with an emphasis on standards compliance and small footprint. Scalable from 8-bit to 32-bit microcontroller environments, the primary governing standards in NuttX are Posix and ANSI standards. Additional standard APIs from Unix and other common RTOSes (such as VxWorks) are adopted for functionality not available under these standards, or for functionality that is not appropriate for deeply-embedded environments (such as fork()). Apache NuttX is an effort undergoing Incubation at The Apache Software Foundation (ASF), sponsored by the Incubator.</p>
        <b>Key features:</b>
        <ul>
            <li>POSIX compliant interface to a high degree.</li>
            <li>Rich Feature OS Set</li>
            <li>Highly scalable</li>
            <li>Real-Time behavior: fully pre-emptible; fixed priority, round-robin, and “sporadic” scheduling.</li>
        </ul>  
        <b>Resources:</b>
        <ul>
            <li><a href="https://nuttx.apache.org/">Official website</a></li>
        </ul>
        <b>**Disclaimer: this tutorial is currently unmantained**</b>
    </div>
  </div>

  <div class="rtositem_image">
    <img src="https://upload.wikimedia.org/wikipedia/commons/b/b0/NuttX_logo.png">
  </div>
</div>


## Bare metal support

The micro-ROS reference boards are the ones officially supported for all RTOSes and with complete support for all available transports.

<div class="rtoscontainer">
  <div class="rtositem_description">
    <h3><b>Arduino bare-metal support</b></h3>
    <div>
        <b>Key features:</b>
        <ul>
            <li>MCU: STM32F407ZGT6 Cortex-M4F</li>
            <li>RAM: 196 kB</li>
            <li>Flash: 1 MB</li>
            <li>Peripherals:  USB OTG, Ethernet, SD Card slot, SPI, CAN, I2C... </li>
        </ul>  
        
        <b>Resources:</b>
        <ul>
            <li><a href="https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware">Official website</a></li>
            <li><a href="https://github.com/OLIMEX/STM32F4/blob/master/HARDWARE/STM32-E407/STM32-E407_Rev_F.pdf">Schematics</a></li>
            <li><a href="https://www.olimex.com/Products/ARM/ST/STM32-E407/resources/STM32-E407.pdf">User Manual</a></li>
        </ul>    
    </div>
  </div>

  <div class="rtositem_image">
    <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/8/87/Arduino_Logo.svg/720px-Arduino_Logo.svg.png">
  </div>
</div>

