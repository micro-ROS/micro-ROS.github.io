---
title: First micro-ROS Application on NuttX
permalink: /docs/tutorials/core/first_application_rtos/nuttx/
redirect_from:
  - /docs/tutorials/advanced/nuttx/nuttx_getting_started/
---

{% include first_application_rtos_common/section_01_intro.md %}

In this tutorial, you'll learn the use of micro-ROS with NuttX.

{% include first_application_rtos_common/section_02_target_hardware_and_workspace.md %}

```bash
# Create step
ros2 run micro_ros_setup create_firmware_ws.sh nuttx olimex-stm32-e407
```

(The created workspace is platform- and RTOS-specific. Use `freertos` or `zephyr` instead of `nuttx` to choose FreeRTOS or Zephyr, respectively.)

{% include first_application_rtos_common/section_03_configuring_firmware.md %}

|  RTOS        | `[APP]`         | `[OPTIONS]`                  |                                  Configured app                                  |
| :------: | --------------- | ---------------------------- | :------------------------------------------------------------------------------: |
|  **NuttX**   | **`uros_pingpong`** |                              | [**Source**](https://github.com/micro-ROS/apps/tree/dashing/examples/uros_pingpong)  |
| FreeRTOS | `ping_pong`     | `--transport serial --dev 3` | [Source](https://github.com/micro-ROS/freertos_apps/tree/dashing/apps/ping_pong)  |
|  Zephyr  | `ping_pong`     | `--transport serial-usb`     |  [Source](https://github.com/micro-ROS/zephyr_apps/tree/dashing/apps/ping_pong)  |

{% include first_application_rtos_common/section_04_demo_description.md %}

Create a new app:

```bash
# Go to app folder inside firmware
cd firmware/apps/examples

# Create your app folder and required files. Contents of these file can be found in column Sample app in table above
mkdir uros_pingpong
cd uros_pingpong
touch Kconfig
touch Makefile
touch app.c
touch Make.defs
```

Create a specific configuration. We're going to start from an already existing one and modify it for our new application.

Execute the following command:
```bash
cd microros_ws
ros2 run micro_ros_setup configure_firmware.sh uros
```

Install required `kconfig-frontends`:

```bash
git clone https://bitbucket.org/nuttx/tools.git firmware/tools

pushd firmware/tools/kconfig-frontends
./configure --enable-mconf --disable-nconf --disable-gconf --disable-qconf 
LD_RUN_PATH=/usr/local/lib && make && sudo make install && sudo ldconfig
popd
```

This sets the Ethernet and micro-ROS required configuration. However, in order to add our application, we're going to modify it:

```bash
cd firmware/NuttX
make menuconfig
```

This will open the NuttX menu config, which allows you to modify the configuration of the RTOS, including adding a new application.

- On the menu, follow the path:
``Application Configuration -> Examples ``
![](../imgs/nuttx_menuconfig.png)

- A list of the available applications will appear. You need to find: ``micro-ROS Ping-Pong`` and click ``y`` to add it.
![](../imgs/nuttx_examples.png)

- Now push three times the key ``ESC`` to close the menu. You will be asked if you want to save your new configuration, and you need to click ``Yes``.

To save your configuration execute the following commands:

```bash
cd uros_ws/firmware/NuttX
make savedefconfig
```

This will generate a file called ``defconfig`` inside of ``uros_ws/firmware/NuttX``. This file is a config profile with all the configuration required to run your specific application.

Finally create a folder called ``uros_pingpong`` into ``uros_ws/firmware/NuttX/configs/olimex-stm32-e407`` and move the defconfig file to uros_pingpong folder so you can execute:

```bash
# Configure step
ros2 run micro_ros_setup configure_firmware.sh uros_pingpong
```

{% include first_application_rtos_common/section_05_building_flashing_and_running.md %}

|   RTOS   | micro-ROS Client to Agent |
| :------: | ------------------------- |
| **NuttX** | **Serial**               |
| FreeRTOS | Serial                    |
|  Zephyr  | USB                       |

{% include first_application_rtos_common/section_06_agent.md %}
