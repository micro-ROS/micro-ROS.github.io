---
title: Micro-ROS 6LoWPAN guide
permalink: /docs/tutorials/advanced/nuttx/6lowpan/
---
This guide will describe how to use Micro-ROS over 6LoWPAN communication.

# What is 6LoWPAN?

6LoWPAN is an acronym of IPv6 over Low-Power Wireless Personal Area Networks.
This communication protocol allows wireless communication over IEEE 802.15.4 based networks using IPv6. 
Some of the main advantages are:

- Easy to route from radio devices to the Internet, thanks to the usage of the IP packets.
- Easy to use on UDP and TCP server/clients.
- A protocol designed for low power and constrained devices. Perfect for Micro-ROS remote sensors.

# Needed devices

At this moment this communication protocol is only available for NuttX RTOS. 
Below the list of what is needed to continue:

- Olimex-STM32-E407 board. [Link](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)
- PmodRF2 Radio. [Link](https://store.digilentinc.com/pmod-rf2-ieee-802-15-rf-transceiver/)
- A Linux Ubuntu 18.04 machine with docker installed.
- A 802.15.4 Radio USB dongle [Link](http://shop.sysmocom.de/products/atusb) plugged into the Linux machine.

# Configure the Olimex board

The configuration of the board is divided into two parts: the set-up of the hardware and the set-up of the software.

## Set-up the hardware

First the PmodRF2 need to be connected to the Olimex board as desribed below:

- `Board D13` -> `MRF24J40 SCLK`
- `Board D12` -> `MRF24J40 MISO`
- `Board D11` -> `MRF24J40 MOSI`
- `Board D10` -> `MRF24J40 CS`,
- `Board D8` -> `MRF24J40 INT`.

The ``D`` letter refers to the Arduino header pins of the board denoted ``CON4``.
To ease the set-up process, The radio's pinout can be found at the following link: [PmodRF2 pinout](https://reference.digilentinc.com/reference/pmod/pmodrf2/start)

The last step is to connect a mini-USB cable to the OTG2 USB port (The USB port next to the Ethernet port).

## Set-up the software

The micro-ROS build system will be use in order to create and flash the
firmware.

Instructions are available at the following link: [Micro-ROS build system](https://github.com/micro-ROS/micro-ros-build/blob/dashing/micro_ros_setup/README.md).
For this particular guide, it is necessary to use the branch ``dashing`` and the configuration profile ``uros_6lowpan``.

### Manual patches

Due to the build system being a work in progress, some of files have to be
modified.

- Open ``uros_ws/firmware/mcu_ws/eProsima/Micro-XRCE-DDS-Client/client.config`` and modify the value of ``CONFIG_UDP_TRANSPORT_MTU`` from ``512`` to ``450``.

This is an appropiate MTU for 6LoWPAN communications on NuttX.
Also the micro-XRCE-DDS needs to be to be change to the version 1.2.5 in the docker:

```bash
cd /uros_ws/firmware/mcu_ws/eProsima/Micro-XRCE-DDS-Client
git checkout v1.2.5
```

Once the modifications done, the binary can be compiled [as explained here](git checkout v1.2.5)

# Configure the Linux machine

The Linux machine is where is going to run the agent.
In order to setup the wireless, some settings have to be set as follow:

```bash
#Setting up the 6lowpan network
sudo apt update && sudo apt install iwpan # install the iwpan tool
sudo ip link set wpan0 down
sudo ip link set lowpan0 down
sudo iwpan dev wpan0 set pan_id 0xabcd
sudo iwpan phy phy0 set channel 0 26 # phy can be phy0 or any other phyN where N [0,1,2,3,4,5,6,7,8,9,10,...]
sudo ip link add link wpan0 name lowpan0 type lowpan
sudo ip link set wpan0 up
sudo ip link set lowpan0 up
sudo ip link set docker0 down # Necessary to not reroute everything over the docker and use the lowpan0
sudo ip neigh flush all
sudo ip neigh add to {**IPV6 of the microROS node**} dev lowpan0 lladdr {**HW 802.15.4 of the micro-controller**} # repeat it for all the device
```

It appears that depending on the PC configuration, the packets will not be able to be routed correctly from the docker to the lowpan0/wpan0 and vice-versa.
In order to forward the packet to the correct interface the following commands need to be entered: 

```bash
#Setting up the 6lowpan routing
sudo ip -6 route add {**IPV6 of the microROS node**} dev lowpan0 proto kernel metric 50 pref medium
```

Once hte setup done, the docker may be run:
```bash
docker run -it --network host microros/micro-ros-agent:dashing udp6 --port 9999  -v6

[1599566641.351706] info     | UDPv6AgentLinux.cpp | init                     | running...             | port: 9999
[1599566641.351889] info     | Root.cpp           | set_verbose_level        | logger setup           | verbose_level: 6

```

# How to use it?

- Olimex-STM32-E407 board must be switched on and a connection to the NSH
  console established,
- By typing ``?`` in the console prompt, ``Builtin Apps``` show be displayed as
  explained below:

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
 - Then the publisher can executed as follow. Note that the ipv6 address (fe80::bc81:c3b9:5c14:1ab) is the
   ipv6 address of the Linux lowpan0 interface.


```bash

nsh> uros_6lowpan fe80::bc81:c3b9:5c14:1ab 9999 pub
Do you want to configure the 6lowpan network? (Y/N)
ifdown wpan0...OK
i8sak: resetting MAC layer
i8sak: starting PAN
Introduce your 6LowPan ID (It must between 00 and FF (Hex))
i8sak: accepting all assoc requests
i8sak: daemon started
ifup wpan0...OK
Connection data
wpan0   Link encap:6LoWPAN HWaddr 00:be:ad:de:00:de:fa:00 at UP
        inet6 addr: fe80::2be:adde:de:fa00/64
        inet6 DRaddr: ::/64

        RX: Received Fragment Errors  
            00000000 00000000 00000000
            IPv6     Dropped 
            00000000 00000000
        TX: Queued   Sent     Errors   Timeouts
            00000000 00000000 00000000 00000000
        Total Errors: 00000000
```
- After this step, the application on the Olimex board will be blocked waiting
  for a user to press any key and enter. The following output should appear.

```bash
Sent: '0'
Sent: '1'
Sent: '2'
...
```

- The same procedure needs to be followed to run the subscriber but the last
  parameter of the ``uros_6lowpan`` command should be ``sub``.
