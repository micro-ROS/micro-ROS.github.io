---
title: UDP echo server
permalink: /docs/tutorials/advanced/nuttx/udp_echo/
---

|  RTOS |  Board Compatible |
|:-----:|:-----------------:|
| NuttX | Olimex-STM32-E407 |

On this tutorial, we will see how to use a UDP echo server on NuttX, to test a basic UDP server. 
UDP (User datagram protocol) is a connectionless protocol with a minimum protocol mechanism. This protocol is focused on time-sensitive applications which is preferable to lose packages rather than delayed packets.

## Hardware requirements:
- [Olimex-STM32-E407 board](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)
- [JTAG Flasher device](https://www.olimex.com/Products/ARM/JTAG/ARM-USB-TINY/)
- Ethernet Cable.
- Mini USB cable.


## Create the firmware

For this tutorial we're going to execute the next configuration on the Micro-ROS build system:
```bash
ros2 run micro_ros_setup create_firmware_ws.sh nuttx olimex-stm32-e407
ros2 run micro_ros_setup configure_firmware.sh udpecho
```

Once the board is configured, we need to build it by typing the next command:
```bash
ros2 run micro_ros_setup build_firmware.sh
```

If the compilation succed, it should return the next output:
```bash
CP: nuttx.hex
CP: nuttx.bin
```

## Flash the firmware
Once the firmware is ready, you need to do the next connections:
- Connect the JTAG flasher device to the JTAG port.
- Connect the mini-USB port to the USB-OTG2 port.
- Connect the ethernet cable to the ethernet socket.

The board it should look like this:
![](images/olimex_ethernet.jpg)

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

## Use the UDP echo server

### NuttX side

Now you need to open the NSH console of NuttX to do so,follow the next steps:

- Push the reset button. The green LED will turn on to say that is working properly.
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
- On this previous example the device is the ``ttyACM0``.
- Open this port with your serial communication app. There are different serial communications applications, but we're goint to use on this tutorial [minicom](https://linux.die.net/man/1/minicom). Type the next command to execute it.
```bash
sudo minicom -D /dev/ttyACM0
```

Once the port is open, **you need to push two time the key enter** and it should pops up the next menu:

```bash
NuttShell (NSH)
nsh> ?
help usage:  help [-v] [<cmd>]

  [         cd        echo      hexdump   mb        nslookup  sh        umount    
  ?         cp        exec      ifconfig  mkdir     ps        sleep     unset     
  arp       cmp       exit      ifdown    mh        pwd       test      usleep    
  basename  dirname   false     ifup      mount     rm        time      xd        
  break     dd        free      kill      mv        rmdir     true      
  cat       df        help      ls        mw        set       uname     

Builtin Apps:
  udpecho  
nsh> 
```
The next step we will check the IP of the Olimex-STM32-E407 board, to do so you need to execute the next commands:
- Mount the PROC fyle system: 
```
mount -t procfs /proc
```
- Execute ifconfig to see the network data:
```
ifconfig
```

This will output:

```bash
nsh> mount -t procfs /proc
nsh> ifconfig
eth0    Link encap:Ethernet HWaddr 00:e0:de:ad:be:ef at UP
        inet addr:192.168.1.130 DRaddr:192.168.1.1 Mask:255.255.255.0

             IPv4   UDP  ICMP
Received     0005  0005  0000
Dropped      0000  0000  0000
  IPv4        VHL: 0000   Frg: 0000
  Checksum   0000  0000  ----
  Type       0000  ----  0000
Sent         0003  0003  0000
nsh> 

```
As you can see the IP address of the ``eth0`` is :192.168.1.130. This is the IP of the board.

**Note**: If you received an incorrect IP or 0 value on the field inet addr, you need to execute the commands:
```
ifdown eth0
ifup eth0
```

The board is now connected to the network and ready to execute the UDP echo server. To execute the application, just type ``udpecho`` and it should output:

```bash
nsh> udpecho
server: 0. Receiving up 1024 bytes
```
## PC side

Open a new console and run a UDP client connected to the port 80 of the NuttX board. For this example we are using netcat, below you can see an example:

```bash
netcat -u 192.168.1.130 80
hello micro-ROS!
hello micro-ROS!
```
When you write something in the UDP client, it should receive the same message and the next output should appear in NuttX:

```bash
server: 0. Received 17 bytes from 192.168.1.132:53394
client: 0. Sending 17 bytes
client: 0. Sent 17 bytes
server: 1. Receiving up 1024 bytes
```
