---
title: Supported Hardware
permalink: /docs/overview/hardware/
---

micro-ROS targets mid-range and high-performance 32-bits microcontrollers families. For now, most of the ports are based on the [STM32 series](https://www.st.com/en/microcontrollers-microprocessors/stm32-32-bit-arm-cortex-mcus.html) from ST. These kinds of MCU feature ARM Cortex-M processors with many peripherals such as GPIO, communication or coprocessors.  By default, micro-ROS uses [NuttX RTOS](https://nuttx.org/), but it also has ports for [FreeRTOS](https://www.freertos.org/) and [Zephyr](https://www.zephyrproject.org/). These RTOSes have a big variety of supported MCUs and development boards. The next list shows some of them:

+ MicroChip PIC32MX Family
+ Atmel SAMA5Dx
+ STM32F1/2/3/4 and STM32L1/4, that includes many Nucleo board support.
+ Renesas M16C/26
+ NXP/Freescale i.MX1/6 and i.MX RT
+ Silicon Labs EFM32 Gecko and Giant Gecko
+ XTENSA board, that includes ESP32
+ RISC-V boards

<!-- TODO (pablogs): Update this paragraph to a more generic porting guide link  -->
Please check [this link](https://cwiki.apache.org/confluence/display/NUTTX/Supported+Platforms) to check the complete list and the status of each board. In case you are interested in porting new boards or MCUs, please check the [next link](https://cwiki.apache.org/confluence/display/NUTTX/Porting+Guide).

Even though many development boards could be used, we have chosen two of them as references. This page lists the hardware platforms that we use to test and develop micro-ROS, and also accessories that we frequently refer to, such as add-on boards, and JTAG probes.

For the ease of use, micro-ROS provides a ready to use examples for some development boards. These out-of-the-box examples aim to show micro-ROS capabilities and they are also a starting point for developing embedded ROS 2 applications.

## Reference Platforms {#evaluation-boards}

This section describes the main characteristic of the selected boards.

## Olimex LTD STM32-E407

![Olimex](https://www.olimex.com/Products/ARM/ST/STM32-E407/images/STM32-E407-02.jpg)

The [Olimex LTD STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware) is an open-hardware low-cost entry board for developing custom applications with the STM32F407ZGT6 Cortex-M4F microcontrollers from STMicroelectronics.

It contains 196KB of RAM and 1MB of Flash. It is a very complete board thanks to the wide variety of communication interfaces it offers: USB OTG, Ethernet, SD Card slot, SPI, CAN or I2C buses are exposed. The board contains various expansion options available: Arduino-like headers for attaching daughter boards, many pins exposed, as well as a UEXT connector. This connector is a custom pin-out bus and is used to attach sensor breakouts sensors that manufacturer sells.

Ports for micro-ROS on all supported RTOS are available for this board. Examples on how to start developing with this board are available:
 - [Zephyr](/docs/tutorials/advanced/zephyr/zephyr_getting_started/)
 - [Nuttx](/docs/tutorials/advanced/nuttx/nuttx_getting_started/)
 - [FreeRTOS](/docs/tutorials/advanced/freertos/freertos_getting_started/)

### Development tools

In order to flash and debug the board, it is required to get a JTAG probe. We recommend getting on of the next JTAG probes:

+ [ARM-USB-OCD-H](https://www.olimex.com/Products/ARM/JTAG/ARM-USB-OCD-H/)
+ [ARM-USB-TINY-H](https://www.olimex.com/Products/ARM/JTAG/ARM-USB-TINY-H/)

### Resources

+ [Vendor main page](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)
+ [Schematics in PDF](https://github.com/OLIMEX/STM32F4/blob/master/HARDWARE/STM32-E407/STM32-E407_Rev_F.pdf)
+ [CAD files](https://github.com/OLIMEX/STM32F4)
+ [User Manual](https://www.olimex.com/Products/ARM/ST/STM32-E407/resources/STM32-E407.pdf)

## STM32L1 Discovery

![](https://www.st.com/content/ccc/fragment/product_related/rpn_information/board_photo/48/0b/aa/9b/b5/d7/43/89/32l152cdiscovery.jpg/files/32l152cdiscovery.jpg/_jcr_content/translations/en.32l152cdiscovery.jpg)

The (STM32L1 Discovery Kit)[https://www.st.com/en/evaluation-tools/32l152cdiscovery.html] is an open hardware design, ultra low power and low-cost entry board for developing custom applications. It contains an STM32L152RCT6 Cortex-M3 microcontroller manufactured by ST Microelectronics. This part number contains 32KB of RAM and 256KB of flash memory. It also includes an ST-Link in-circuit debugger that allows flashing and debugging the target microcontroller.

### Resources

+ [Vendor main page](https://www.st.com/en/evaluation-tools/32l152cdiscovery.html)
+ [User Manual](https://www.st.com/content/ccc/resource/technical/document/user_manual/08/f8/63/f5/7b/3d/40/ff/DM00027954.pdf/files/DM00027954.pdf/jcr:content/translations/en.DM00027954.pdf)
+ [Schematics in PDF](https://www.st.com/resource/en/schematic_pack/32l152cdiscovery_sch.zip)
+ [Bill of Material](https://www.st.com/resource/en/bill_of_materials/32l152cdiscovery_bom.zip)
+ [Gerber files](https://www.st.com/resource/en/bill_of_materials/32l152cdiscovery_bom.zip)


## STM32L4 Discovery kit IoT 

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


## Crazyflie 2.1 Drone

As an integration example, the open-source [Crazyflie 2.1](https://www.bitcraze.io/products/crazyflie-2-1/) platform has its own micro-ROS + FreeRTOS port. 

<img width="400" src="imgs/3.jpg">

The Crazyflie 2.1 is a versatile open-source flying development platform that only weighs 27g and fits in the palm of your hand. Crazyflie 2.1 is equipped with multiple inertial sensors and low-latency/long-range radio as well as Bluetooth LE.

This little drone features a STM32F405 ARM Cortex-M4 MCU running up to 168 MHz with 1 MB of Flash and 192 KB of RAM. It also features the following sensors and coprocessors:
 - nRF51822 radio and power management MCU (Cortex-M0, 32Mhz, 16kb SRAM, 128kb flash)
 - USB
 - LiPo battery charger
 - 8KB EEPROM
 - 3-axis accelerometer and gyroscope (BMI088)
 - pressure sensor (BMP388)
 - headers with peripheral access: SPI, I2C, UART, 1-wire and GPIO

Examples on how to start developing with this board are available [here](/docs/tutorials/demos/crazyflie_demo/).