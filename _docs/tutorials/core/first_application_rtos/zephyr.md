---
title: First micro-ROS Application on Zephyr
permalink: /docs/tutorials/core/first_application_rtos/zephyr/
redirect_from:
  - /docs/tutorials/advanced/zephyr/zephyr_getting_started/
---

## Target platform

In this tutorial, you'll learn the use of micro-ROS with Zephyr by testing a Ping Pong application.

{% include first_application_common/target_hardware.md %}
* [USB-to-mini-USB cable](https://www.olimex.com/Products/Components/Cables/CABLE-USB-A-MINI-1.8M/)

{% include first_application_common/build_system.md %}

```bash
# Create step
ros2 run micro_ros_setup create_firmware_ws.sh zephyr olimex-stm32-e407
```

Once the command is executed, a folder named `firmware` must be present in your workspace.

This step is in charge, among other things, of creating a set of micro-ROS apps for the specific platform you are
addressing.
In the case of Zephyr, these are located at `firmware/zephyr_apps/apps`.
Each app is represented by a folder containing the following files:

* `src/main.c`: This file contains the logic of the application.
* `app-colcon.meta`: This file contains the micro-ROS app specific colcon configuration. Detailed info on how to
  configure the RMW via this file can be found
  [here](https://micro-ros.github.io/docs/tutorials/core/microxrcedds_rmw_configuration/).
* `CMakeLists.txt`: This is the CMake file containing the script to compile the application.
* `prj.conf`: This is a Zephyr specific app configuration file.

For the user to create its custom application, a folder `<my_app>` will need to be registered in this location,
containing the four files just described.

{% include first_application_common/config.md %}

In this tutorial, we will use a USB transport (labeled as `serial-usb`) and focus on the out-of-the-box `ping_pong`
application located at `firmware/zephyr_apps/apps/ping_pong`. To execute this application with the chosen transport,
run the configuration command above by specifying the `[APP]` and `[OPTIONS]` parameters as below:

```bash
# Configure step with ping_pong app and serial-usb transport
ros2 run micro_ros_setup configure_firmware.sh ping_pong --transport serial-usb
```
You can check the complete content of the `ping_pong` app
[here](https://github.com/micro-ROS/zephyr_apps/tree/dashing/apps/ping_pong).

{% include first_application_common/pingpong_logic.md %}

The contents of the Zephyr app specific files can be found here:
[main.c](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/ping_pong/src/main.c),
[app-colcon.meta](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/ping_pong/app-colcon.meta),
[CMakeLists.txt](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/ping_pong/CMakeLists.txt)
and [prj.conf](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/ping_pong/prj.conf).
A thorough review of these files is illustrative of how to create a micro-ROS app in this RTOS.

{% include first_application_common/build_and_flash.md %}

{% include first_application_common/agent_creation.md %}

Then, depending on the selected transport and RTOS, the board connection to the agent may differ.
In this tutorial, we're using the Olimex STM32-E407 USB connection, for which the Olimex development board is connected
to the computer using the USB OTG 2 connector (the miniUSB connector that is furthest from the Ethernet port).

<img width="400" style="padding-right: 25px;" src="../imgs/6.jpg">

{% include first_application_common/run_app.md %}

{% include first_application_common/test_app.md %}
