---
title: Micro-ROS 6LoWPAN Guide 
permalink: /docs/tutorials/advanced/microros_6lowpan_tutorial/
---

In this guide, we will show how to use Micro-ROS over 6LoWPAN communication.

# What is 6LoWPAN?

6LoWPAN is an acronym of IPv6 over Low-Power Wireless Personal Area Networks. This communication protocol allows wireless communication over IEEE 802.15.4 based networks using IPv6.  Some of the main advantages are:
- Easy to route from radio devices to the Internet, thanks to the usage of the IP packets.
- Easy to use on UDP and TCP server/clients.
- A protocol designed for low power and constrained devices. Perfect for Micro-ROS remote sensors.

# Needed devices

At this moment this communication protocol is only available for NuttX RTOS. You need the following devices to implement the steps highlighted in this guide:

- Olimex-STM32-E407 board. [Link](https://www.olimex.com/Products/ARM/ST/STM32-E407/open-source-hardware)
- PmodRF2 Radio. [Link](https://store.digilentinc.com/pmod-rf2-ieee-802-15-rf-transceiver/)
- Micro-ROS hardware bridge.

**Important!**
You can find a guide of how to set-up the Micro-ROS hardware bridge on the next link: [uROS hardware bridge guide](https://github.com/micro-ROS/micro-ROS-bridge_RPI/blob/new_bridge_tools/Readme.md)
On the micro-ROS hardware bridge guide, you can find everything that you need to set-up this device base.

# Configure the board

The configuration of the board is divided into two parts: the set-up of the hardware and the set-up of the software.

## Set-up the hardware
First we're going to connect the Pmodrf2 radio. To do so, follow the next wiring table:

- `Board D13` -> `MRF24J40 SCLK`
- `Board D12` -> `MRF24J40 MISO`
- `Board D11` -> `MRF24J40 MOSI`
- `Board D10` -> `MRF24J40 CS`
- `Board D8` -> `MRF24J40 INT`

The ``D`` letter, reference to the Arduino header pins of the board.
To help on the set-up process, you can find the pinout of the radio on the next link: [PmodRF2 pinout](https://reference.digilentinc.com/reference/pmod/pmodrf2/start)

The last point is to connect a mini-USB cable to the OTG2 USB port (The USB port next to the Ethernet port).

Let's continue with the next step
## Set-up the software

To create and flash the firmware, we're going to use the micro-ROS build system.

You can find the instructions on the next link: [Micro-ROS build system](https://github.com/micro-ROS/micro-ros-build/blob/dashing/micro_ros_setup/README.md).
For this particular guide, it is necessary to use the branch ``dashing`` and use the configuration profile ``uros_6lowpan``.

Once you follow all the instructions on the build system, and the board is flashed everything is ready.

# How to use it?

- Turn on the Olimex-STM32-E407 board and open the NSH console on a terminal.
- Check if all the applications are ready by typing ``?`` on the console. It should then return the next:

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
- The value **inet6** is the IPV6 direction of the Raspberry Pi over the 6LoWPAN interface. Copy it, because is necessary on further steps.

- Go to the Olimex-STM32-E407 board and execute the micro-ROS 6lowpan application, by typing the next command: ``uros_6lowpan <Agent_IP> <Agent_Port> <pub/sub>
  - Agent_IP: In this field, you should write the IP of the Agent, on this specific example, the IP that you copy on the previous step.
  - Agent_PORT: Write the port that you want to open for micro-ROS communications.
  - pub/sub: If you write **pub**, it will create a micro-ROS publisher, which will publish an integer numbers up to one thousand. Otherwise, if you write **sub** it will create a subscriber that will be subscribe to the ``std_msgs_msg_Int32`` topic.

- Once you execute the app, it will ask you if you want to configure the 6LoWPAN network. (Necessary to configure each time that you reboot the board)
  - This will return connection data, you should save the ``inet_6_addr`` and ``HWaddr``.
  - Note: If you want to change the ID of the radio, you can do it on the menuconfig of NuttX on the example configuration.

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
- After this step, the application on the Olimex board will be blocked waiting for a user input confirming that the Agent on the hardware bridge is ready to receive data.

- Come back to the hardware bridge, now we're going to add a new 6lowpan device. Push ``1 + enter``. (Note: This step is only necessary if you're attaching for the first time a new device)
  - First, introduce the IPV6 of the Olimex board (inet6_addr)
  - Introduce the hardware address of the Olimex board (Hwaddr).
  - Now the device is registered and ready to establish communication.
- The final step on the hardware bridge is to execute the micro-ROS Agent, to do so, push ``2 + enter`` and introduce the port to use.

- Finally, on the Olimex board, push ``y`` to confirm that everything is ready and it will start the micro-ROS application.
  
