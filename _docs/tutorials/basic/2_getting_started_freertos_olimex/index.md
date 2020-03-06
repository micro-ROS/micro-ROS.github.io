This tutorial aims to setup a Hello World micro-ROS application on **[Olimex STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)** evaluation board with **[FreeRTOS RTOS](https://www.freertos.org/)**


*TODO: Link to micro-ROS explanation (e.g Agent)*

*TODO: Image of the board and FreeRTOS logo*

## Required hardware

This tutorial uses the following hardware:

| Item | |
|---------------|----------------------------------------------------------|
| Olimex STM32-E407 | [Link](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware) |
| Olimex ARM-USB-TINY-H | [Link](https://www.olimex.com/Products/ARM/JTAG/ARM-USB-TINY-H/) |
| USB-Serial Cable Female | [Link](https://www.olimex.com/Products/Components/Cables/USB-Serial-Cable/USB-Serial-Cable-F/) |


Olimex STM32-E407 features a STM32F407 Cortex-M4 microcontroller. It has 1MB Flash and 196 kB RAM of which 64 kB are core coupled (CCM SRAM). Other features of the chip are:

- Three 12 bits @ 2.4MHz ADC
- Two 12 bits DAC
- USB OTG FS
- USB OTG HS
- Ethernet
- 14 timers
- 3 SPI
- 3 I2C
- 2 CAN
- 114 GPIOs

Most of this peripherals are mapped into Olimex board headers.

The micro-ROS + FreeRTOS port of this evaluation board relies on a STM32CubeMX generated project, so HAL and low-level configuration can be tunned to specific requirements.

The out-of-the-box HAL configuration included in the port packages configures the minimal communication and debugging peripheral required for micro-ROS.

## Running micro-ROS

First of all make sure that you have a **ROS 2 Dashing** installation.

***TIP:** if you are familiar with Docker containers, this image may be useful: [ros:dasing](https://hub.docker.com/layers/ros/library/ros/dashing/images/sha256-b796c14ea663537129897769aa6c715a851ca08dffd4875ef2ecaa31a4dbd431?context=explore)*

On the **ROS 2 Dashing** installation open a command line and follow this steps:

```bash
# Source the ROS 2 Dashing installation
source /opt/ros/$ROS_DISTRO/setup.bash

# Create a workspace and download the micro-ROS tools
mkdir microros_ws 
cd microros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro-ros-build.git src/micro-ros-build

# Update dependencies using rosdep
rosdep update
rosdep install --from-path src --ignore-src -y

# Build micro-ROS tools and source them
colcon build
source install/local_setup.bash
```

Now you have all the requiered tools to crosscompile micro-ROS and FreeRTOS for Olimex STM32-E407 development board. At this point, you must know that the micro-ROS build system is a four-step workflow:

1. **Create**: retrieve all the required packages for a specific RTOS and hardware platform.
2. **Configure**: configures the downloaded packages with options such as the micro-ROS application, the selected transport layer or the micro-ROS agent IP address (in network transports).
3. **Build**: generate a binary file ready for being loaded in the hardware.
4. **Flash**: load the micro-ROS software in the hardware.

So, lets create a firmware workspace for the target system:

```bash
# Create step
ros2 run micro_ros_setup create_firmware_ws.sh freertos olimex-stm32-e407
```

Once we have the firmware packages ready, lets configure the app to the `simple_publisher` sample and a serial transport on UART3.

```bash
# Configure step
ros2 run micro_ros_setup configure_firmware.sh olimex_simple_publisher --transport serial --dev 3
```

You can check the sample app [code here](https://github.com/micro-ROS/freertos_apps/blob/dashing/apps/olimex_simple_publisher/app.c).

When the configuring step ends, just build the firmware:

```bash
# Build step
ros2 run micro_ros_setup build_firmware.sh
```

Once the build has successfully ended, lets power and connect the board. 

First, connect Olimex ARM-USB-TINY-H JTAG programmer to the board's JTAG port:

*TODO: JTAG image*

Make sure that the board power supply jumper (PWR_SEL) is in the 3-4 position in order to power the board from the JTAG connector:

*TODO: jumper image*

```bash
# Flash step
ros2 run micro_ros_setup flash_firmware.sh
```

## Test the app

The micro-ROS app is ready to connect to a micro-ROS Agent and start talking with the rest of the ROS 2 world.

First of all, create and build a micro-ROS agent:

```bash
# Download micro-ROS Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build micro-ROS Agent packages, this may take a while.
colcon build
```

Then connect the Olimex development board to the computer using the usb to serial cable:

*TODO: usb serial image*

Then run the agent:

```bash
# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev [device]
```

***TIP:** you can use this command to find your serial device name: `ls /dev/serial/by-id/*`*

And finally, check that everything is working:

```bash
# Subscribe to micro-ROS sample app topic
ros2 topic echo /olimex/publisher
```



<!-- [![demo](https://asciinema.org/a/113463.svg)](https://asciinema.org/a/113463?autoplay=1) -->
