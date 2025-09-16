---
title: Integration into External Tools
permalink: /docs/overview/ext_tools/
---

<style>
.extplatcontainer {
  height: 300px;
  display: flex;
  flex-direction: row;
  justify-content: flex-start;
  flex-wrap: wrap;
}

.extplatitem_image {
  width: 35%;
  display: flex;
  align-items: center;
  justify-content: center;
}

.extplatitem_description {
  width: 65%;
}

.extplatitem_image img {
    max-width: 100%;
}
</style>

micro-ROS aims to **bring ROS 2 to microcontrollers** to allow having first-class ROS 2 entities in the embedded world.

One of the approaches offered by micro-ROS to build an application for embedded platforms consists in a [ROS-specific build system](https://github.com/micro-ROS/micro_ros_setup) comprising modules which integrate the software for cross-compiling said apps on the supported plaforms, both hardware and firmware-wise. A different approach consists in generating standalone modules and components allowing to integrate micro-ROS into external or custom development frameworks, made possible by a [tool dedicated to compiling micro-ROS as a standalone library](../../tutorials/advanced/create_custom_static_library).

The configuration of the generated micro-ROS libraries is based on a `colcon.meta` file.

The modules that exist up to date for integrating into external build systems are the following:

### **Vulcanexus micro-ROS element**

<div class="extplatcontainer">
  <div class="extplatitem_description">
    <div>
      <a href="https://vulcanexus.org">Vulcanexus</a> is an all-in-one ROS 2 tool set for easy and customized robotics development. It offers natively integrated solutions for ROS 2 networks in terms of performance improvement, simulation, cloud/edge communication, and microcontroller integration. The latter relies on micro-ROS. Vulcanexus is free and open source, and available as a Docker image for Humble and Iron. All components enjoy continuous updates so users benefit from the latest features at all times.
    </div>
  </div>
</div>

### **micro-ROS component for the ESP-IDF**

<div class="extplatcontainer">
  <div class="extplatitem_description">
    <div>
        ESP-IDF is the official development framework for the ESP32, ESP32-S and ESP32-C Series SoCs.
        To date, it has been tested in ESP-IDF v4.1 and v4.2 with ESP32 and ESP32-S2.
        The <i>micro-ROS component for the ESP-IDF</i> allows the user to integrate the micro-ROS API and utilities in an already created ESP-IDF project just by cloning or copying a folder.
        The current ports support <a href="https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/uart.html">Serial (UART)</a>,
        <a href="https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_wifi.html">WiFi</a>, and <a href="https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/index.html#ethernet">Ethernet</a>.
        <hr><b>Resources:</b>
        <ul>
            <li><a href="https://github.com/micro-ROS/micro_ros_espidf_component">micro-ROS component for ESP-IDF on GitHub</a></li>
            <li><a href="https://github.com/espressif/esp-idf">ESP-IDF on GitHub</a></li>
            <li><a href="https://docs.espressif.com/projects/esp-idf/en/latest/esp32/">ESP-IDF docs</a></li>
        </ul>
    </div>
  </div>

  <div class="extplatitem_image">
    <img src="https://www.espressif.com/sites/all/themes/espressif/logo-black.svg">
  </div>
</div>

### **micro-ROS module for the Zephyr build system**

<div class="extplatcontainer">
  <div class="extplatitem_description">
    <div>
        Zephyr is a scalable RTOS built with security in mind, and based on a small-footprint kernel designed for use on resource-constrained systems.
        The Zephyr kernel supports multiple hardware architectures, including ARM Cortex-M, Intel x86, ARC, Nios II, Tensilica Xtensa, and RISC-V, and can count with <a href="https://docs.zephyrproject.org/latest/boards/index.html">large number of supported boards</a>.
        The <i>micro-ROS module for Zephyr</i> allows to integrate the micro-ROS API and utilities in an existing Zephyr-based project just by cloning or copying a folder.
        <hr><b>Resources:</b>
        <ul>
            <li><a href="https://github.com/micro-ROS/micro_ros_zephyr_module">micro-ROS module for Zephyr build system on GitHub</a></li>
            <li><a href="https://github.com/zephyrproject-rtos/zephyr">Primary GitHub repository for the Zephyr Project</a></li>
            <li><a href="https://github.com/zephyrproject-rtos/zephyr">Zephyr Project official webpage</a></li>
        </ul>
    </div>
  </div>

  <div class="extplatitem_image">
    <img src="/img/posts/logo-zephyr.jpg">
  </div>
</div>

### **micro-ROS for Arduino**

<div class="extplatcontainer">
  <div class="extplatitem_description">
    <div>
        Arduino is an open-source platform based on an I/O board and a development environment that implements the Processing/Wiring language, intended to enable users to easily generate interactive projects. A CLI is also offered, which aims to be an all-in-one solution providing the tools needed to use any Arduino compatible platform from the command line.
        The <i>micro-ROS for Arduino</i> support package is a special <i>bare-metal</i> port of micro-ROS provided as a set of precompiled libraries for specific platforms.
        <hr><b>Resources:</b>
        <ul>
            <li><a href="https://github.com/micro-ROS/micro_ros_arduino">micro-ROS for Arduino on GitHub</a></li>
            <li><a href="https://github.com/arduino/Arduino">Arduino IDE on GitHub</a></li>
            <li><a href="https://github.com/arduino/arduino-cli">Arduino CLI on GitHub</a></li>
            <li><a href="https://www.arduino.cc/">Arduino official website</a></li>
        </ul>
    </div>
  </div>

  <div class="extplatitem_image">
    <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/8/87/Arduino_Logo.svg/720px-Arduino_Logo.svg.png" width="190">
  </div>
</div>

### **micro-ROS for STM32CubeMX**


<div class="extplatcontainer">
  <div class="extplatitem_description">
    <div>
        The STM32CubeMX is a graphical tool by ST for configuring STM32 microcontrollers and microprocessors. It enables to optimally program and manipulate the software thanks to a set of utilities that help setting up pinouts, peripherals, and middleware stacks.
        <i>micro-ROS for STM32CubeMX</i> can be configured using the static library builder <a href="https://hub.docker.com/r/microros/micro_ros_static_library_builder/">docker image</a>  and allows micro-ROS to be virtually supported by the full set of boards offered by <a href="https://www.st.com">STMicroelectronics</a>, in turn enabling the seamless integration of micro-ROS into any STM32 controller based project.
        <hr><b>Resources:</b>
        <ul>
            <li><a href="https://github.com/micro-ROS/micro_ros_stm32cubemx_utils">micro-ROS for STM32CubeMX on GitHub</a></li>
            <li><a href="https://www.st.com">STMicroelectronics official webpage</a></li>
            <li><a href="https://github.com/STMicroelectronics/STM32Cube_MCU_Overall_Offer">STMicroelectronics official webpage</a></li>
        </ul>
    </div>
  </div>

  <div class="extplatitem_image">
    <img src="https://www.pinclipart.com/picdir/big/453-4531945_read-more-stm32cubemx-logo-clipart.png" width="190">
  </div>
</div>

{% include logos_disclaimer.md %}
