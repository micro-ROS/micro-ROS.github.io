---
title: First micro-ROS Application on Zephyr
permalink: /docs/tutorials/core/first_application_rtos/zephyr/
redirect_from:
  - /docs/tutorials/advanced/zephyr/zephyr_getting_started/
---

<img src="https://img.shields.io/badge/Tested_on-Humble-green" style="display:inline"/> <img src="https://img.shields.io/badge/Tested_on-Rolling-green" style="display:inline"/> <img src="https://img.shields.io/badge/Tested_on-Iron-green" style="display:inline"/>

In this tutorial, you'll learn the use of micro-ROS with Zephyr by testing a Ping Pong application.
{% include first_application_common/target_hardware.md %}
* [USB-to-mini-USB cable](https://www.olimex.com/Products/Components/Cables/CABLE-USB-A-MINI-1.8M/)

{% include first_application_common/build_system.md %}

```bash
# Create step
ros2 run micro_ros_setup create_firmware_ws.sh zephyr olimex-stm32-e407
```

{% include first_application_common/zephyr_common.md %}

{% include first_application_common/config.md %}

In this tutorial, we will use a USB transport (labeled as `serial-usb`) and focus on the out-of-the-box `ping_pong`
application located at `firmware/zephyr_apps/apps/ping_pong`. To execute this application with the chosen transport,
run the configuration command above by specifying the `[APP]` and `[OPTIONS]` parameters as below:

```bash
# Configure step with ping_pong app and serial-usb transport
ros2 run micro_ros_setup configure_firmware.sh ping_pong --transport serial-usb
```
You can check the complete content of the `ping_pong` app
[here](https://github.com/micro-ROS/zephyr_apps/tree/humble/apps/ping_pong).

{% include first_application_common/pingpong_logic.md %}

The contents of the Zephyr app specific files can be found here:
[main.c](https://github.com/micro-ROS/zephyr_apps/blob/humble/apps/ping_pong/src/main.c),
[app-colcon.meta](https://github.com/micro-ROS/zephyr_apps/blob/humble/apps/ping_pong/app-colcon.meta),
[CMakeLists.txt](https://github.com/micro-ROS/zephyr_apps/blob/humble/apps/ping_pong/CMakeLists.txt)
and [serial-usb.conf](https://github.com/micro-ROS/zephyr_apps/blob/humble/apps/ping_pong/serial-usb.conf).
A thorough review of these files is illustrative of how to create a micro-ROS app in this RTOS.

{% include first_application_common/build_and_flash.md %}

{% include first_application_common/agent_creation.md %}

Then, depending on the selected transport and RTOS, the board connection to the agent may differ.
In this tutorial, we're using the Olimex STM32-E407 USB connection, for which the Olimex development board is connected
to the computer using the USB OTG 2 connector (the miniUSB connector that is furthest from the Ethernet port).

<img width="400" style="padding-right: 25px;" src="../imgs/6.jpg">

{% include first_application_common/run_app.md %}

{% include first_application_common/test_app_rtos.md %}

This completes the First micro-ROS Application on Zephyr tutorial. Do you want to [go back](../) and try a different RTOS, i.e. NuttX or FreeRTOS?
