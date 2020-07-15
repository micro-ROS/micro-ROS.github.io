---
title: First micro-ROS Application on FreeRTOS
permalink: /docs/tutorials/core/first_application_rtos/freertos/
redirect_from:
  - /docs/tutorials/advanced/freertos/freertos_getting_started/
---

{% include first_application_rtos_common/section_01_intro.md %}

In this tutorial, you'll learn the use of micro-ROS with FreeRTOS.

{% include first_application_rtos_common/section_02_target_hardware_and_workspace.md %}

```bash
# Create step
ros2 run micro_ros_setup create_firmware_ws.sh freertos olimex-stm32-e407
```

{% include first_application_rtos_common/section_03_configuring_firmware.md %}

|  RTOS        | `[APP]`         | `[OPTIONS]`                  |                                  Configured app                                  |
| :------: | --------------- | ---------------------------- | :------------------------------------------------------------------------------: |
|  NuttX   | `uros_pingpong` |                              | [Source](https://github.com/micro-ROS/apps/tree/dashing/examples/uros_pingpong)  |
| **FreeRTOS** | **`ping_pong`**     | **`--transport serial --dev 3`** | [**Source**](https://github.com/micro-ROS/freertos_apps/tree/dashing/apps/ping_pong)  |
|  Zephyr  | `ping_pong`     | `--transport serial-usb`     |  [Source](https://github.com/micro-ROS/zephyr_apps/tree/dashing/apps/ping_pong)  |

{% include first_application_rtos_common/section_04_demo_description.md %}

Create a new app:

```bash
# Create your app folder and required files. Contents of these file can be found in column Sample app in table above
pushd firmware/freertos_apps/apps
mkdir ping_pong
cd ping_pong
touch app.c app-colcon.meta
popd
```

Now you are ready to call:

```bash
# Configure step
ros2 run micro_ros_setup configure_firmware.sh ping_pong [OPTIONS]
```

{% include first_application_rtos_common/section_05_building_flashing_and_running.md %}

|   RTOS   | micro-ROS Client to Agent |
| :------: | ------------------------- |
|  NuttX   | Serial                    |
| **FreeRTOS** | **Serial**            |
|  Zephyr  | USB                       |

{% include first_application_rtos_common/section_06_agent.md %}

This completes the First micro-ROS Application on FreeRTOS tutorial. Do you want to [go back](../) and try a different RTOS, i.e. NuttX or Zephyr?
