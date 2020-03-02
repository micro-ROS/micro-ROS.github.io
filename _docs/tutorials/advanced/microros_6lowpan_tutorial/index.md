---
title: Micro-ROS 6LoWPAN Guide 
permalink: /docs/tutorials/advanced/microros_6lowpan_tutorial/
---

On this guide we will show how to use Micro-ROS over 6LoWPAN communication.

# What is 6LoWPAN?

6LoWPAN is acronym base of IPv6 over Low -Power Wireless Personal Area Networks. This communication protocol allows wireless communication over IEEE 802.15.4 based network using IPv6.  Some of the main advantage are:
- Easy to route from radio devices to the Internet, thanks to ussage of the IP packets.
- Easy to use on UDP and TCP server/clients.
- Protocol designed for low power and constrain devices. Perfect for Micro-ROS remote sensors.

# What do you need to try?

At this moment this communication protocol is only available for NuttX RTOS. You need the next devices to try this guide:

- Olimex-STM32-E407 board. [Link](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)
- PModRF2 Radio [Link](https://store.digilentinc.com/pmod-rf2-ieee-802-15-rf-transceiver/)
- Micro-ROS hardware bridge.

**Important!**
You can find a guide of how to set-up the Micro-ROS hardware bridge on the next link: [uROS hardware bridge guide](https://github.com/micro-ROS/micro-ROS-bridge_RPI/blob/new_bridge_tools/Readme.md)
On the micro-ROS hardware bridge guide you can find everything that you need to set-up this device base.

# Configure the board

This point will split into set-up the hardware and set-up the software

## Set-up the hardware
First we're going to connect the Pmodrf2 radio. To do so, follow the next wiring table:

- `Board D13` -> `MRF24J40 SCLK`
- `Board D12` -> `MRF24J40 MISO`
- `Board D11` -> `MRF24J40 MOSI`
- `Board D10` -> `MRF24J40 CS`
- `Board D8` -> `MRF24J40 INT`

The ``D`` reference to the Arduino header pins of the board.
To help on the set-up process, you can find the pinout of the radio on the next link: [PmodRF2 pinout](https://reference.digilentinc.com/reference/pmod/pmodrf2/start)

The last point, is connect a mini-USB cable to the OTG2 USB port (The USB port next to the Ethernet port).

From the hardware point, everything is ready.
## Set-up the software

To create and flash the firmware, we're going to use the micro-ROS build system.

You can find the instructions on the next link: [Micro-ROS build system](https://github.com/micro-ROS/micro-ros-build/blob/dashing/micro_ros_setup/README.md).
For this particular guide, is necessary to use the branch ``dashing`` and use the configuration profiel ``uros_6lowpan``.

Once you follow all the instructions on the build system, and the board is flashed everything is ready.

# How to use it?

- Turn on the Olimex-STM32-E407 board and open the NSH console on a terminal.
- Check if all the applications are ready by typing ``?`` on the console. It should the return the next:
```bash
help usage:  help [-v] [<cmd>]

  [         cd        df        help      ls        mw        set       true      
  ?         cp        echo      hexdump   mb        ps        sh        uname     
  addroute  cmp       exec      ifconfig  mkdir     pwd       sleep     umount    
  basename  dirname   exit      ifdown    mh        rm        test      unset     
  break     dd        false     ifup      mount     rmdir     telnetd   usleep    
  cat       delroute  free      kill      mv        route     time      xd        

Builtin Apps:
  ping6         i8sak         uros_6lowpan 
```
- Now turn-on the Raspberry Pi and execute the micro-ROS hardware bridge tool, by typing the next command:
```bash
./ ~/micro-ROS-HB.sh
```
- Once everything is configured, it will return the connection data of the 6LowPAN network:
```bash
lowpan0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1280
        inet6 fe80::b482:ca65:743b:b6bd  prefixlen 64  scopeid 0x20<link>
        unspec B6-82-CA-65-74-3B-B6-BD-00-00-00-00-00-00-00-00  txqueuelen 1000  (UNSPEC)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 21  bytes 2242 (2.1 KiB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

1) Add new 6LoWPAN micro-ROS device	 3) Create UDP micro-ROS Agent		  5) Create Serial micro-ROS Agent server
2) Create UDP 6LoWPAN micro-ROS Agent	 4) Create TCP micro-ROS Agent		  6) Quit
#? 

```
- Copy the value **inet6**, this will usefull on the next steps, because the IPV6 direcction of the RPI on the 6LoWPAN network.


