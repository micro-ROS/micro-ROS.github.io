---
title: TCP echo server
permalink: /docs/tutorials/advanced/nuttx/tcp_echo/
---

|  RTOS |  Board Compatible |
|:-----:|:-----------------:|
| NuttX | Olimex-STM32-E407 |

In this tutorial, we will see how to use a TCP echo server on NuttX, to test a basic TCP server. TCP (Transmission Control Protocol) is a connection-oriented protocol, and a connection between client and server is established (passive open) before data can be sent.

## Hardware requirements:
- [Olimex-STM32-E407 board](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)
- [JTAG Flasher device](https://www.olimex.com/Products/ARM/JTAG/ARM-USB-TINY/)
- Ethernet Cable.
- Mini USB cable.


## Create the firmware

For this tutorial we're going to execute the following configuration in the Micro-ROS build system:
```bash
ros2 run micro_ros_setup create_firmware_ws.sh nuttx olimex-stm32-e407
ros2 run micro_ros_setup configure_firmware.sh tcpecho
```

Once the board is configured, we need to build it by typing the command:
```bash
ros2 run micro_ros_setup build_firmware.sh
```

If the compilation succeds, it should return this output:
```bash
CP: nuttx.hex
CP: nuttx.bin
```

## Flash the firmware
Once the firmware is ready, you need to do the following connections:
- Connect the JTAG flasher device to the JTAG port.
- Connect the mini-USB port to the USB-OTG2 port.
- Connect the ethernet cable to the ethernet socket.

The board should look like this:
![](images/olimex_ethernet.jpg)

Now flash the board by typing the command:
```bash
ros2 run micro_ros_setup flash_firmware.sh
```

This should return this output once the process is finished:
```bash
wrote 49152 bytes from file nuttx.bin in 6.279262s (7.644 KiB/s)
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
```
## Use the TCP echo server

### NuttX side

Now you need to open the NSH console of NuttX. To do so, follow these steps:

- Push the reset button. The green LED will turn on to say that it is working properly.
- Look for the device by typing ``dmesg`` on the console. This should return something like this:
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
- Open this port with your serial communication app. There are different serial communication applications, but in this tutorial we're going to use [minicom](https://linux.die.net/man/1/minicom). Type the next command to execute it.
```bash
sudo minicom -D /dev/ttyACM0
```

Once the port is open, **you need to push two times the Enter key** and the next menu should pop up:

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
  tcpecho  
nsh> 
```
In the following step we will check the IP of the Olimex-STM32-E407 board. To do so you need to execute the commands below:
- Mount the PROC fyle system: 
```
mount -t procfs /proc
```
- Execute ifconfig to see the network data:
```
ifconfig
```

This will return as an output:

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

**Note**: If you received an incorrect IP or 0 value on the field inet addr, you need to execute the next commands:
```
ifdown eth0
ifup eth0
```

The board is now connected to the network and ready to execute the TCP echo server. To execute the application, just type ``tcpecho`` and it should return the next output:

```bash
nsh> tcpecho
Start TCP echo server
```
## PC side

Open a new console to run a TCP client. Connect it to the port 80 of the NuttX board. For this example we are using ``netcat``, on the bottom line you can see an example:

```bash
netcat 192.168.1.130 80
hello micro-ROS!
hello micro-ROS!
```
When you write something on the TCP client, if everything goes fine, it should receive the same message that you send.
