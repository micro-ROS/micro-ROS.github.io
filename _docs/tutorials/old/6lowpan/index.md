---
title: 6LoWPAN Guide
permalink: /docs/tutorials/old/6lowpan/
---

<img src="https://img.shields.io/badge/Disclaimer-This_tutorial_is_unmaintained-red" style="display:inline"/>

In this guide, we will show how to use micro-ROS over 6LoWPAN communication.

## What is 6LoWPAN?

6LoWPAN is an acronym o IPv6 over Low-Power Wireless Personal Area Networks.
This communication protocol allows wireless communication over IEEE 802.15.4 based networks using IPv6. Some of the main advantages are:
- Easy to route from radio devices to the Internet, thanks to the usage of the IP packets.
- Easy to use on UDP and TCP server/clients.
- A protocol designed for low power and constrained devices, perfect or micro-ROS remote sensors.

## Needed hardware

At present, 6LoWPAN is only available for the NuttX RTOS.
In order to implement the steps highlighted in this guide, you need the following devices:

- Raspberry Pi.
- [Olimex-STM32-E407 board](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware).
- [PmodRF2 Radio](https://store.digilentinc.com/pmod-rf2-ieee-802-15-rf-transceiver/).
- micro-ROS-bridge-RPI.

**Important!**
You can find a guide of how to setup the micro-ROS-bridge_RPI at its [repository](https://github.com/micro-ROS/micro-ROS-bridge_RPI/blob/new_bridge_tools/README.md).
In the micro-ROS-bridge-RPI guide, you can find everything that you need to set-up this device base.

## Configure the board

The configuration of the board is divided into two parts: hardware and software set-up.

### Hardware set-up

First we are going to connect the PmodRF2 radio. 

|       | RPi | Olimex | PmodRF2 |
| ----- | --- | ------ | ------- |
| VIN   | 1   | D13    | 12      |
| GND   | 20  | GND    | 11      |
| RESET | 17  | -      | 8       |
| INT   | 16  | D8     | 7       |
| SDI   | 19  | D12    | 2       |
| SDO   | 21  | D11    | 3       |
| SCK   | 23  | D13    | 4       |
| CS    | 26  | D10    | 1       |

To ease the set-up process, you can use the [RPi pinout](https://pinout.xyz/#) and [PmodRF2 pinout](https://reference.digilentinc.com/reference/pmod/pmodrf2/start).

The last step is to connect a mini-USB cable to the OTG2 USB port (this USB port next to the Ethernet port).

### Software set-up

To create and flash the firmware, we are going to use the micro-ROS build system.
You can find the instructions at the micro_ros_setup's [README](https://github.com/micro-ROS/micro_ros_setup/blob/dashing/micro_ros_setup/README.md).
For this particular guide, it is necessary to use the branch `dashing` and the configuration profile `uros_6lowpan`.

Once you follow all the instructions in the build system and flash the board, everything is ready.

## How to use it?

- Turn on the Olimex board and open the NSH console on a terminal.
- Check if all the applications are ready by typing `?` on the console. It should return the following:

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
- Now turn-on the RPi and execute the micro-ROS-bridge-RPI tool by typing the next command:

```bash
./ ~/micro-ROS-HB.sh
```
- Once everything is configured, it will return the connection data of the 6LoWPAN network:

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
- The value **inet6** is the IPv6 direction of the RPi over the 6LoWPAN interface. Copy it because is necessary for the subsequent steps.
- Execute the micro-ROS 6LoWPAN application on the Olimex typing the next command:

```bash
uros_6lowpan <Agent_IP> <Agent_Port> <pub/sub>
```
  where `Agent_IP` is the IPv6 copied previously, `Agent_PORT` is the port selected for the Agent and `pub/sub` controls the application behavior: in case of `pub` it will act as publisher and as in case of `sub`, it will act as a subscriber.

- Once you execute the app, ti will ask you if you want to configure the 6LoWPAN network.
  - This will return connection data, you should save the `inet_6_addr` and `HWaddr`.
  - Note: if you want to change the ID of the radio, you can do it on the `menuconfig` of NuttX on the example configuration.

```bash
nsh> uros_6lowpan fe80::bc81:c3b9:5c14:1ab 8888 pub
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

- After this step, the application on the Olimex board will be blocked waiting for a user input confirming that the Agent on the bridge is ready to receive data.

- Go back to the bridge, now we are going to add a new 6LoWPAN device. Push `1 + enter`. (Note: this step is only necessary if you are attaching for the first time a new device).
  - First, introduce the IPv6 of the Olimex board (`inet6_addr`).
  - Introduce the hardware address of the Olimex board (HWaddr).
  - Now the device is registered and ready to establish communication.
- The final step on the bridge is to execute the micro-ROS-Agent, to do so, push `2 + enter` and introduce the port to use.
