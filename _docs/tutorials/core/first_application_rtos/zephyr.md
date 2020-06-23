---
title: First micro-ROS Application on Zephyr
permalink: /docs/tutorials/core/first_application_rtos/zephyr/
redirect_from:
  - /docs/tutorials/advanced/zephyr/zephyr_getting_started/
---

{% include first_application_rtos_common/section_01_intro.md %}

In this tutorial, you'll learn the use of micro-ROS with Zephyr.

{% include first_application_rtos_common/section_02_target_hardware_and_workspace.md %}

```bash
# Create step
ros2 run micro_ros_setup create_firmware_ws.sh zephyr olimex-stm32-e407
```

**NOTE for Zephyr: Make sure you have the latest version of CMake!**

```bash
sudo apt install wget
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | sudo apt-key add -
sudo apt install software-properties-common
sudo apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'
sudo apt update
sudo apt install cmake
```

(The created workspace is platform- and RTOS-specific. Use `nuttx` or `freertos` instead of `zephyr` to choose NuttX or FreeRTOS, respectively.)

{% include first_application_rtos_common/section_03_configuring_firmware.md %}

|    RTOS    | `[APP]`         | `[OPTIONS]`                  |                                   Configured app                                   |
| :--------: | --------------- | ---------------------------- | :--------------------------------------------------------------------------------: |
|   NuttX    | `uros_pingpong` |                              |  [Source](https://github.com/micro-ROS/apps/tree/dashing/examples/uros_pingpong)   |
|  FreeRTOS  | `ping_pong`     | `--transport serial --dev 3` |  [Source](https://github.com/micro-ROS/freertos_apps/tree/dashing/apps/ping_pong)  |
| **Zephyr** | **`ping_pong`** | **`--transport serial-usb`** | [**Source**](https://github.com/micro-ROS/zephyr_apps/tree/dashing/apps/ping_pong) |

{% include first_application_rtos_common/section_04_demo_description.md %}

micro-ROS apps for Zephyr are located at `firmware/zephyr_apps/apps`. In order to create a new application, create a new folder containing two files: the app code (inside a `src` folder) and the RMW configuration. You will also need some other Zephyr related files: a `CMakeLists.txt` to define the building process and a `prj.conf` where Zephyr is configured. There is a sample proyect [here](https://github.com/micro-ROS/zephyr_apps/tree/dashing/apps/ping_pong), for now, it is ok to copy them.

Create a new app:

```bash
# Create your app folder and required files. Contents of these file can be found in column Sample app in table above
pushd firmware/zephyr_apps/apps
mkdir ping_pong
cd ping_pong
mkdir src
touch src/app.c app-colcon.meta
touch CMakeLists.txt prj.conf
popd
```

The contents of the files can be found here: [app.c](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/ping_pong/src/main.c), [app-colcon.meta](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/ping_pong/app-colcon.meta), [CMakeLists.txt](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/ping_pong/CMakeLists.txt) and [prj.conf](https://github.com/micro-ROS/zephyr_apps/blob/dashing/apps/ping_pong/prj.conf).

Once the app folder is created, let's configure our new app with a UDP transport that looks for the agent on the port UDP/8888 at localhost:

```bash
# Configure step
ros2 run micro_ros_setup configure_firmware.sh ping_pong --transport serial-usb
```

{% include first_application_rtos_common/section_05_building_flashing_and_running.md %}

|    RTOS    | micro-ROS Client to Agent |
| :--------: | ------------------------- |
|   NuttX    | Serial                    |
|  FreeRTOS  | Serial                    |
| **Zephyr** | **USB**                   |

{% include first_application_rtos_common/section_06_agent.md %}

This completes the First micro-ROS Application on Zephyr tutorial. Do you want to [go back](../) and try a different RTOS, i.e. NuttX or FreeRTOS?
