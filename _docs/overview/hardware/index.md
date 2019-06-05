---
title: Supported Hardware
permalink: /docs/overview/hardware/
---

This page lists the hardware platforms that we use to test and develop micro-ROS, and also accessories that we frequently refer to, such as add-on boards, and JTAG probes.

## Reference Platforms {#evaluation-boards}

Even that micro-ROS technology can be ported to many targets, we have decided to select development platforms to support them. This page describes the main characteristic of those boards.

## Olimex LTD STM32-E407

![Olimex](https://www.olimex.com/Products/ARM/ST/STM32-E407/images/STM32-E407-02.jpg)

The [Olimex LTD STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware) is an open-hardware low-cost entry board for developing custom applications with the STM32F407ZGT6 Cortex-M4F microcontrollers from STMicroelectronics.

It contains 196KB of RAM and 1MB of Flash. It is a very complete board thanks to the wide variety of communication interfaces it offers: USB OTG, Ethernet, SD Card slot, SPI, CAN or I2C buses are exposed. The board contains various expansion options available: Arduino-like headers for attaching daughter boards, many pins exposed, as well as a UEXT connector. This connector is a custom pin-out bus and is used to attach sensor breakouts sensors that manufacturer sells.

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
