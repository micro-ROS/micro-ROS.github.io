## Building the firmware

When the configuring step ends, just build the firmware:

```bash
# Build step
ros2 run micro_ros_setup build_firmware.sh
```

## Flashing the firmware

Flashing the firmware into the platform varies across hardware platforms.
Regarding this tutorial's target platform
(**[Olimex STM32-E407](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)**),
the JTAG interface is going to be used to flash the firmware.

Connect the [Olimex ARM-USB-TINY-H](https://www.olimex.com/Products/ARM/JTAG/ARM-USB-TINY-H/) to the board:

<img width="400" style="padding-right: 25px;" src="../imgs/2.jpg">

Make sure that the board power supply jumper (PWR_SEL) is in the 3-4 position in order to power the board from the
JTAG connector:

<img width="400" style="padding-right: 25px;" src="../imgs/1.jpg">

Once you have your computer connected to the Olimex board through the JTAG adapter, run the flash step:

```bash
# Flash step
ros2 run micro_ros_setup flash_firmware.sh
```