The target hardware for this tutorial is the **[Olimex STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)** evaluation board.

First of all, make sure that you have a **ROS 2** environment and the **micro-ROS build system** installed in the computer. If not, please check the [**First micro-ROS application on Linux**](../../first_application_linux/) tutorial in order to learn how to start with micro-ROS.

The build system's workflow in the case of embedded systems is a four-step procedure:

* **Create step:** This step is in charge of downloading all the required code repositories and cross-compilation toolchains for the specific hardware platform. Among these repositories, it will also download a collection of ready to use micro-ROS apps.
* **Configure step:** In this step, the user can select which app is going to be cross-compiled by the toolchain. Some other options, such as transport, agent address or port will be also selected in this step.
* **Build step:** Here is where the cross-compilation takes place and the platform-specific binaries are generated.
* **Flash step:** The binaries generated in the previous step are flashed onto the hardware platform memory, in order to allow the execution of the micro-ROS app.

Further information about micro-ROS build system can be found [here](https://github.com/micro-ROS/micro-ros-build/tree/dashing/micro_ros_setup).

## Required hardware

The following hardware will be used:

* [Olimex STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)
* [Olimex ARM-USB-TINY-H](https://www.olimex.com/Products/ARM/JTAG/ARM-USB-TINY-H/)
* [USB-Serial Cable Female](https://www.olimex.com/Products/Components/Cables/USB-Serial-Cable/USB-Serial-Cable-F/)

## Step 1: Creating a new firmware workspace

In order to accomplish the first step, a new firmware workspace can be created using the command: