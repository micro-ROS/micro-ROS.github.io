---
title: NSH console over USB
permalink: /docs/tutorials/advanced/nuttx/nsh_usb/
---

|  RTOS |  Board Compatible |
|:-----:|:-----------------:|
| NuttX | Olimex-STM32-E407 |

NSH is a system console that can be used through the following interfaces: USB,UART and Telnet. With this console you can execute apps, set system configurations and see the state of the system.

## Hardware requirements:
- [Olimex-STM32-E407 board](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)
- [JTAG Flasher device](https://www.olimex.com/Products/ARM/JTAG/ARM-USB-TINY/)
- Mini USB cable.


## Create the firmware

For this tutorial we're going to execute the following configuration in the Micro-ROS build system:
```bash
ros2 run micro_ros_setup create_firmware_ws.sh nuttx olimex-stm32-e407
ros2 run micro_ros_setup configure_firmware.sh nsh
```

Once the board is configured, we need to build it by typing the next:
```bash
ros2 run micro_ros_setup build_firmware.sh
```

If the compilation succeds, it should return this output:
```bash
CP: nuttx.hex
CP: nuttx.bin
```
## Flash the firmware

Now you're ready to upload the firmware to the board. Connect the JTAG flasher device to the board and the mini USB cable to the USB-OTG2 port like in the image below.
![](images/olimex_nsh_usb.jpg)

Now flash the board by typing the next command:
```bash
ros2 run micro_ros_setup flash_firmware.sh
```

This should return this output once the process is finished:
```bash
wrote 49152 bytes from file nuttx.bin in 6.279262s (7.644 KiB/s)
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
```
## Connect to the console

Finally, to use the NSH console you need to follow the steps listed below:
- Push the reset button. The green LED will turn on to say that it is working properly.
- Look for the device by typing ``dmesg`` on the console, this should return somenthing like this:
```bash
[20614.570781] usb 1-2: new full-speed USB device number 7 using xhci_hcd
[20614.724366] usb 1-2: New USB device found, idVendor=0525, idProduct=a4a7, bcdDevice= 1.01
[20614.724372] usb 1-2: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[20614.724375] usb 1-2: Product: CDC/ACM Serial
[20614.724378] usb 1-2: Manufacturer: NuttX
[20614.724381] usb 1-2: SerialNumber: 0
[20614.745693] cdc_acm 1-2:1.0: ttyACM0: USB ACM device
[20614.746274] usbcore: registered new interface driver cdc_acm
[20614.746277] cdc_acm: USB Abstract Control Model driver for USB modems and ISDN adapters
```
- In this example the device is the ``ttyACM0``.
- Open this port with your serial communication app. There are different serial communication applications, but in this tutorial we're going to use [minicom](https://linux.die.net/man/1/minicom). Type the following command to execute it.
```bash
sudo minicom -D /dev/ttyACM0
```

Once the port is open, **you need to push two times the Enter key** and it should the menu below should pop up:
```bash
nsh> ?
help usage:  help [-v] [<cmd>]

  ?           exec        hexdump     mb          sleep       
  cat         exit        kill        mh          usleep      
  echo        help        ls          mw          xd          

Builtin Apps:
nsh>
```
