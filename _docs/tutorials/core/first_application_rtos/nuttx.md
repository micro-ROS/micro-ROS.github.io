---
title: First micro-ROS Application on NuttX
permalink: /docs/tutorials/core/first_application_rtos/nuttx/
redirect_from:
  - /docs/tutorials/advanced/nuttx/nuttx_getting_started/
---

<img src="https://img.shields.io/badge/Tested_on-Humble-green" style="display:inline"/>

In this tutorial, you'll learn the use of micro-ROS with NuttX by testing a Ping Pong application.
{% include first_application_common/target_hardware.md %}
* [USB-to-Serial Cable Female](https://www.olimex.com/Products/Components/Cables/USB-Serial-Cable/USB-SERIAL-F/)
* [USB-to-mini-USB cable](https://www.olimex.com/Products/Components/Cables/CABLE-USB-A-MINI-1.8M/)

{% include first_application_common/build_system.md %}

```bash
# Create step
ros2 run micro_ros_setup create_firmware_ws.sh nuttx olimex-stm32-e407
```

Once the command is executed, a folder named `firmware` must be present in your workspace.

This step is in charge, among other things, of downloading a set of micro-ROS apps for the specific platform you are
addressing.
In the case of NuttX, these are located [here](https://github.com/micro-ROS/nuttx_apps/tree/foxy/examples).
Each app is represented by a folder containing the following files:

* `app.c`: This file contains the logic of the application.
* `Kconfig`: This file contains the NuttX Kconfig configuration.
* `Make.defs`: This file contains the	NuttX build system definitions.
* `Makefile`: This file contains the NuttX specific app build script.

## Configuring the firmware

The configuration step will set up the main micro-ROS options and select the desired application.
It can be executed with the following command:

```bash
# Configure step
ros2 run micro_ros_setup configure_firmware.sh [APP] [OPTIONS]
```

In this tutorial, we will use a Serial transport and focus on the out-of-the-box `uros_pingpong`
application located [here](https://github.com/micro-ROS/nuttx_apps/tree/foxy/examples/uros_pingpong).
To execute this application with the chosen transport, run the configuration command above by specifying the `[APP]` parameter as below:

```bash
# Configure step with ping_pong app and serial-usb transport
ros2 run micro_ros_setup configure_firmware.sh pingpong
```

and with no `[OPTIONS]` parameter.

A pre-configured ethernet example is also available:
```bash
# Configure step with ping_pong app and serial-usb transport
ros2 run micro_ros_setup configure_firmware.sh pingpong-eth
```

To proceed with the configuration, clone the following NuttX tools repo:

```bash
# Download the tools necessary to work with NuttX
git clone https://bitbucket.org/nuttx/tools.git firmware/tools
```

and then install the required `kconfig-frontends`:

```bash
pushd firmware/tools/kconfig-frontends
./configure
make

# if the make command fails, type: autoreconf -f -i , and then rerun the make command.

sudo make install
sudo ldconfig
popd
```

Now we have two options to configure our micro-ROS transport:

- Interactive NuttX menu config
  * Launch the configuration menu:

    ```bash
    cd firmware/NuttX
    make menuconfig
    ```

  * You can check that the application has been selected under `Application Configuration ---> Examples ---> micro-ROS Ping Pong`.
  * The transport is also pre-configured under the `Application Configuration ---> micro-ROS ---> Transport` option.
  * To configure the transport, use the `IP address of the agent` and `Port number of the agent` options for UDP and `Serial port to use` for the serial example.
  * To save the changes, navigate to the bottom menu with the left and right arrows, and click on the `Save` button.
  * You will be asked if you want to save your new `.config` configuration, and you need to click `Ok`, and then `Exit`.
  * Push three times the `Esc` key to close the menu and go back to `microros_ws` with:

      ```bash
      cd ../..
      ```

- `kconfig-tweak` console commands:
  * Go to Nuttx configuration path:

    ```bash
    cd firmware/NuttX
    ```

  * UDP transport configuration:
    ```bash
    kconfig-tweak --set-val CONFIG_UROS_AGENT_IP "127.0.0.1"
    kconfig-tweak --set-val CONFIG_UROS_AGENT_PORT 8888
    ```

  * Serial transport configuration:
    ```bash
    kconfig-tweak --set-val CONFIG_UROS_SERIAL_PORT "/dev/ttyS0"
    ```

You can check the complete content of the `uros_pingpong` app
[here](https://github.com/micro-ROS/nuttx_apps/tree/foxy/examples/uros_pingpong).

{% include first_application_common/pingpong_logic.md %}

The contents of the FreeRTOS app specific files can be found here:
[app.c](https://github.com/micro-ROS/nuttx_apps/blob/foxy/examples/uros_pingpong/app.c),
[Kconfig](https://github.com/micro-ROS/nuttx_apps/blob/foxy/examples/uros_pingpong/Kconfig),
[Make.defs](https://github.com/micro-ROS/nuttx_apps/blob/foxy/examples/uros_pingpong/Make.defs) and
[Makefile](https://github.com/micro-ROS/nuttx_apps/blob/foxy/examples/uros_pingpong/Makefile).
A thorough review of these files is illustrative of how to create a micro-ROS app in this RTOS.

{% include first_application_common/build_and_flash.md %}

{% include first_application_common/agent_creation.md %}

Then, depending on the selected transport and RTOS, the board connection to the agent may differ.
In this tutorial, we're using the Olimex STM32-E407 Serial connection, for which the Olimex development board is
connected to the computer using the usb to serial cable.

<img width="400" style="padding-right: 25px;" src="../imgs/5.jpg">

Additionally, you'll need to connect a USB-to-mini-USB cable to the USB OTG 1 connector (the miniUSB connector
that is closer to the Ethernet port).

<img width="500" style="padding-right: 25px;" src="../imgs/7.jpg">

***TIP:** Color codes are applicable to
[this cable](https://www.olimex.com/Products/Components/Cables/USB-Serial-Cable/USB-SERIAL-F/).
Make sure to match Olimex Rx with Cable Tx and vice-versa. Remember GND!*

## Running the micro-ROS app

At this point, you have both the client and the agent correctly installed.

To give micro-ROS access to the ROS 2 dataspace, run the agent:

```bash
# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev [device]
```

***TIP:** you can use this command to find your serial device name: `ls /dev/serial/by-id/*`*

Then, in order to launch the micro-ROS application, you need to install and open Minicom,
a text-based serial port communications program. Open a new shell, and type:

```bash
sudo minicom -D [device] -b 115200
```

***TIP:** you can use this command to find your serial device name: `ls /dev/serial/by-id/*`. Select the one that starts with `usb-NuttX`.*

From inside the Minicom application, press three times the `Enter` key until Nuttx Shell (NSH) appears.
Once you enter the NSH command line, type:

```bash
uros_pingpong
```

{% include first_application_common/test_app_rtos.md %}

This completes the First micro-ROS Application on NuttX tutorial. Do you want to [go back](../) and try a different RTOS, i.e. FreeRTOS or Zephyr?
