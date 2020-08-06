---
title: First micro-ROS Application on FreeRTOS
permalink: /docs/tutorials/core/first_application_rtos/freertos/
redirect_from:
  - /docs/tutorials/advanced/freertos/freertos_getting_started/
---

## Target platform

In this tutorial, you'll learn the use of micro-ROS with FreeRTOS by testing a Ping Pong application.
{% include first_application_common/target_hardware.md %}
* [USB-to-Serial Cable Female](https://www.olimex.com/Products/Components/Cables/USB-Serial-Cable/USB-SERIAL-F/)

{% include first_application_common/build_system.md %}

```bash
# Create step
ros2 run micro_ros_setup create_firmware_ws.sh freertos olimex-stm32-e407
```

Once the command is executed, a folder named `firmware` must be present in your workspace.

This step is in charge, among other things, of downloading a set of micro-ROS apps for the specific platform you are
addressing.
In the case of FreeRTOS, these are located at `firmware/freertos_apps/apps`.
Each app is represented by a folder containing the following files:

* `app.c`: This file contains the logic of the application.
* `app-colcon.meta`: This file contains the micro-ROS app specific colcon configuration. Detailed info on how to
  configure the RMW via this file can be found
  [here](https://micro-ros.github.io/docs/tutorials/core/microxrcedds_rmw_configuration/).

For the user to create its custom application, a folder `<my_app>` will need to be registered in this location,
containing the two files just described.

{% include first_application_common/config.md %}

In this tutorial, we will use a Serial transport (labeled as `serial`) and focus on the out-of-the-box `ping_pong`
application located at `firmware/freertos_apps/nuttx_apps/ping_pong`. To execute this application with the chosen transport,
run the configuration command above by specifying the `[APP]` and `[OPTIONS]` parameters as below:

```bash
# Configure step with ping_pong app and serial-usb transport
ros2 run micro_ros_setup configure_firmware.sh ping_pong --transport serial --dev 3
```

You can check the complete content of the `ping_pong` app
[here](https://github.com/micro-ROS/freertos_apps/tree/foxy/nuttx_apps/ping_pong).

{% include first_application_common/pingpong_logic.md %}

The contents of the FreeRTOS app specific files can be found here:
[app.c](https://github.com/micro-ROS/freertos_apps/blob/foxy/nuttx_apps/ping_pong/app.c) and
[app-colcon.meta](https://github.com/micro-ROS/freertos_apps/blob/foxy/nuttx_apps/ping_pong/app-colcon.meta).
A thorough review of these files is illustrative of how to create a micro-ROS app in this RTOS.

{% include first_application_common/build_and_flash.md %}

{% include first_application_common/agent_creation.md %}

Then, depending on the selected transport and RTOS, the board connection to the agent may differ.
In this tutorial, we're using the Olimex STM32-E407 Serial connection, for which the Olimex development board is
connected to the computer using the usb to serial cable.

<img width="400" style="padding-right: 25px;" src="../imgs/5.jpg">

***TIP:** Color codes are applicable to
[this cable](https://www.olimex.com/Products/Components/Cables/USB-Serial-Cable/USB-SERIAL-F/).
Make sure to match Olimex Rx with Cable Tx and vice-versa. Remember GND!*

{% include first_application_common/run_app.md %}

{% include first_application_common/test_app_rtos.md %}

This completes the First micro-ROS Application on FreeRTOS tutorial. Do you want to [go back](../) and try a different RTOS, i.e. NuttX or Zephyr?
