---
title: Supported Hardware
permalink: /docs/concepts/rtos/supported_hardware/
---

micro-ROS targets mid-range and high-performance 32-bits microcontrollers families. For now, most of the ports are based on the [STM32 series](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html) from ST. These kinds of MCU feature ARM Cortex-M processors with many peripherals such as GPIO, communication or coprocessors.

For the ease of use, micro-ROS provides a ready to use examples for some development boards. These out-of-the box examples aim to show micro-ROS capabilities and they are also a starting point for developing embedded ROS 2 applications.

# Olimex STM32-E407

[Olimex STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware) is the micro-ROS **reference board**. Ports for micro-ROS on all supported RTOS are available for this board.

<img width="400" src="imgs/1.jpg">

STM32-E407 is a low-cost entry board for developing custom applications with **STM32F407ZGT6 Cortex M4** microcontrollers from ST. This board has plenty of resources and all GPIO ports are available on extension connectors, an Arduino compatible platform connector is also available.

This board features an STM32F407 Cortex-M4 with 1 MB of Flash memory and 196 KB of RAM. Of the overall amount of RAM, 64 KB are Core Coupled Memory (CCM), a special section of RAM focus on high-speed transactions.

Its main features include: 
 - three 12 bits Analog-to-Digital converters
 - two 12 bits Digital-to-Analog converters
 - USB
 - Ethernet 
 - 14 timers
 - three SPI ports 
 - three I2C ports 
 - two CAN bus ports
 - micro-SD card slot
 - programmable user LED
 - programmable user push button
 - Arduino compatible header
 - on-board 6 to 16 VDC power supply
 - up to 114 GPIOS

Examples on how to start developing with this board are available:
 - [Zephyr](/docs/tutorials/advanced/zephyr/zephyr_getting_started/)
 - [Nuttx](/docs/tutorials/advanced/nuttx/nuttx_getting_started/)
 - [FreeRTOS](/docs/tutorials/advanced/freertos/freertos_getting_started/)

# STM32L4 Discovery kit IoT 

The [ST B-L475E-IOT01A](https://www.st.com/en/evaluation-tools/b-l475e-iot01a.html) evaluation board is a ready to use IoT kit. This board supports an out-of-the-box micro-ROS + Zephyr port.

<img width="400" src="imgs/2.jpg">

The STM32L4 Discovery kit IoT enables a wide diversity of applications by exploiting low-power communication, multiway sensing and Arm Cortex M4 core-based STM32L4 Series features.
The support for Arduino and PMOD connectivity provides unlimited expansion capabilities with a large choice of specialized add-on boards.

This board features a STM32L475E MCU with 1 MB of Flash memory and 128 KB of RAM. In addition to the MCU peripherals, the board includes: 
 - 64 Mb SPI Flash memory
 - Bluetooth V4.1 module (SPBTLE-RF)
 - 915 MHz low-power RF module (SPSGRF-915)
 - 802.11 b/g/n module (ISM43362-M3G-L44)
 - NFC tag based on M24SR with printed antenna
 - 2 digital microphones (MP34DT01)
 - relative humidity and temperature digital sensor (HTS221)
 - 3-axis magnetometer (LIS3MDL)
 - 3-axis accelerometer and gyroscope (LSM6DSL)
 - digital barometer (LPS22HB)
 - Time-of-Flight and gesture-detection sensor (VL53L0X)
 - programmable push-buttons
 - USB OTG FS with Micro-AB connector
 - on-board ST-LINK/V2 debugger and programmer

Examples on how to start developing with this board are available [here](/docs/tutorials/demos/tof_demo/).

# Crazyflie 2.1 Drone

As an integration example, the open-source [Crazyflie 2.1](https://www.bitcraze.io/products/crazyflie-2-1/) platform has its own micro-ROS port. 

<img width="400" src="imgs/3.jpg">

The Crazyflie 2.1 is a versatile open source flying development platform that only weighs 27g and fits in the palm of your hand. Crazyflie 2.1 is equipped with low-latency/long-range radio as well as Bluetooth LE.

This little drone features a STM32F405 ARM Cortex-M4 MCU running up to 168 MHz with 1 MB of Flash and 192 KB of RAM. It also features the following sensors and coprocessors:
 - nRF51822 radio and power management MCU (Cortex-M0, 32Mhz, 16kb SRAM, 128kb flash)
 - USB
 - LiPo battery charger
 - 8KB EEPROM
 - 3-axis accelerometer and gyroscope (BMI088)
 - pressure sensor (BMP388)
 - headers with peripheral access: SPI, I2C, UART, 1-wire and GPIO

Examples on how to start developing with this board are available [here](/docs/tutorials/demos/crazyflie_demo/).
