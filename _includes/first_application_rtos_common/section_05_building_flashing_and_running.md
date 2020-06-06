## Step 3: Building the firmware

When the configuring step ends, just build the firmware:

```bash
# Build step
ros2 run micro_ros_setup build_firmware.sh
```

## Step 4: Flashing the firmware

Flashing the firmware into the platform may vary across hardware platforms. Regarding this tutorial's target platform (**[Olimex STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)**), JTAG interface is going to be used to flash the firmware.

Connect [Olimex ARM-USB-TINY-H](https://www.olimex.com/Products/ARM/JTAG/ARM-USB-TINY-H/) to the board:

<img width="400" style="padding-right: 25px;" src="../imgs/2.jpg">

Make sure that the board power supply jumper (PWR_SEL) is in the 3-4 position in order to power the board from the JTAG connector:

<img width="400" style="padding-right: 25px;" src="../imgs/1.jpg">

Once you have your computer connected to the Olimex board through the JTAG adapter, run the flash step:

```bash
# Flash step
ros2 run micro_ros_setup flash_firmware.sh
```

## Running the micro-ROS app

The micro-ROS app is ready to connect to a micro-ROS-Agent and start talking with the rest of the ROS 2 world.

First of all, create and build a micro-ROS agent:

```bash
# Download micro-ROS-Agent packages
ros2 run micro_ros_setup create_agent_ws.sh

# Build micro-ROS-Agent packages, this may take a while.
colcon build
source install/local_setup.bash
```

Then, depending on the selected transport and RTOS, the board connection to the agent may differ:
