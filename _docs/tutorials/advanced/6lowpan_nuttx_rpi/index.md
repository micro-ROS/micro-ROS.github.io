---
title: 6LOWPAN communications between NuttX and a Raspberry Pi
permalink: /docs/tutorials/advanced/6lowpan_nuttx_rpi/
---


This guide will show how to set-up a Raspberry Pi 3 (RPI) running Raspbian and an Olimex STM32 E407 board running NuttX to have 6lowpan communication between them.

## What do we need?

- Raspberry Pi 3
- micro-SD card (almost 16gb) with Raspbian lite already installed.
- Olimex-STM32-E407 board.
- Two PMODRF2 module which are base on the MRF24J40 module.
- PC with Ubuntu (It works fine with Ubuntu 16.04)
- NuttX source code, you can find a Docker file with all the tools [here](https://github.com/micro-ROS/docker/tree/pre_refactor/Embedded/NuttX/development/stm32-e407).
## How to set-up 6lowpan RPI?

First, we need to connect the PMODRF2 module to the RPI, so we need to set the next connections:

|  | RPI | PMODRF2 |
| -- | -- | -- |
| VIN | 1 | 12 |
| GND | 20 | 11 |
| RESET | 17 | 8 |
| INT | 16 | 7 |
| SDI | 19 | 2 |
| SDO | 21 | 3 |
| SCK | 23 | 4 |
| CS | 26 | 1 |

In the next links you can see the pinout of each board:
- [Raspberry Pi 3 pinout.](https://i.pinimg.com/originals/84/46/ec/8446eca5728ebbfa85882e8e16af8507.png)
- [PMODRF2 pionut.](https://reference.digilentinc.com/reference/pmod/pmodrf2/start)

Once you've set all the wires, power-on the RPI and download the next repository inside the RPI:
- https://github.com/micro-ROS/micro-ROS-bridge_RPI

Execute the next command:
```sudo ./micro-ROS-bridge_RPI/RPI_6lowpan/script.sh```

If everything goes fine, at the end of the script, the board should restart. After the start-up, type the next command to see if the configuration and the connections are fine:
```dmesg | grep mrf24j40```

And if everything is fine, it should return the next:
```bash
pi@raspberrypi:~ $ dmesg | grep mrf24j40
[    4.075212] mrf24j40 spi0.0: probe(). IRQ: 169
```
The last point is to set-up the network.

- Set the PAN ID: ``sudo iwpan dev wpan0 set pan_id 0xabcd``
- Set the page and channel: ``sudo iwpan phy phy0 set channel 0 26``
- Set the short address: ``sudo iwpan dev wpan0 set short_addr 0x4204`` (Each board must a different one)
- Attach the phy layer to the lowpan: ``sudo ip link add link wpan0 name lowpan0 type lowpan``
- Bring up the WPAN0 interface: ``sudo ip link set wpan0 up``
- Bring up the RPI lowpan: ``sudo ip link set lowpan0 up``


Now the RPI is ready to send and receive messages from a NuttX board or another RPI.


## How to set-up 6lowpan in NuttX?

First, we need to do the connections between the [Olimex board](/docs/overview/hardware) and the PMODRF2 module.

- `Board D13` -> `MRF24J40 SCLK`
- `Board D12` -> `MRF24J40 MISO`
- `Board D11` -> `MRF24J40 MOSI`
- `Board D10` -> `MRF24J40 CS`
- `Board D8` -> `MRF24J40 INT`

Once the wiring is finished, we need to compile and upload the firmware. Type the next commands:

Go to the main folder of NuttX and type the command to configure the board:
`./scripts/configure.sh olimex-stm32-e407 mrf24j40-6lowpan`

Compile:
`make`

Upload:
`./scripts/flash.sh olimex-stm32-e407`

It should return somenthing like this:

- Configuration:
```
Copy files
Refreshing...
```
- Compilation:
```
CP: nuttx.hex
CP: nuttx.bin
```
- Upload the firmware
```
wrote 131072 bytes from file nuttx.bin in 3.763846s (34.008 KiB/s)
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
```
(The number of written bytes could be different in each board)


Finally, connect the mini USB cable to the USB OTG2, and the console should return the next:

```bash
nsh> ?
help usage:  help [-v] [<cmd>]

  [         cd        df        help      ls        mw        set       true      
  ?         cp        echo      hexdump   mb        ps        sh        uname     
  addroute  cmp       exec      ifconfig  mkdir     pwd       sleep     umount    
  basename  dirname   exit      ifdown    mh        rm        test      unset     
  break     dd        false     ifup      mount     rmdir     telnetd   usleep    
  cat       delroute  free      kill      mv        route     time      xd        

Builtin Apps:
  udp_6lowpan  ping6        i8sak
```

We're going to configure the network. Execute `udp_6lowpan`.
The program will ask you if you want to configure the network. Type **Y** to start the configuration process.

**Important note: If you don't configure the network, the connection won't be possible with other boards.**

Then will ask you if you want to set this board as a coordinator or as a node.
The difference between a coordinator and a node is that the first one can work as a router, coordinating the network traffic of up to 8 nodes.
On the other hand, the node is an endpoint device which only sends and receive data, it doesn't coordinate the traffic of the other devices.

For this example, we will set ``coordinator``.

Finally will ask for an ID. This ID must be unique for each board. If everything goes fine, this should return the next:

```bash
Your hardware address is: i8sak set eaddr 00:fa:de:00:de:ad:be:00

i8sak: accepting all assoc requests
i8sak: daemon started
ifup wpan0...OK
Mounting proc file system
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




Available commands
 -To send a package type: write
 -To receive a package type: read
 -To exit type: quit

```

**At this point the network is ready to work!**

Finally type ``quit`` two times to close the app and come back to the main menu.

## Sending a message from NuttX to Raspbian:

### Raspberry Part:

We need to check our IP, so type in the console: ```ifconfig```
This should return something like this:

```bash
eth0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.1.173  netmask 255.255.255.0  broadcast 192.168.1.255
        inet6 fe80::47b1:3e6c:17fa:a06a  prefixlen 64  scopeid 0x20<link>
        ether b8:27:eb:69:5b:e7  txqueuelen 1000  (Ethernet)
        RX packets 538496  bytes 394976364 (376.6 MiB)
        RX errors 0  dropped 2416  overruns 0  frame 0
        TX packets 358285  bytes 51814460 (49.4 MiB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

lo: flags=73<UP,LOOPBACK,RUNNING>  mtu 65536
        inet 127.0.0.1  netmask 255.0.0.0
        inet6 ::1  prefixlen 128  scopeid 0x10<host>
        loop  txqueuelen 1000  (Local Loopback)
        RX packets 48  bytes 7076 (6.9 KiB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 48  bytes 7076 (6.9 KiB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

lowpan0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1280
        inet6 fe80::a9cd:ff:fe00:4204  prefixlen 64  scopeid 0x20<link>
        inet6 fe80::9c6e:87a5:eb60:84d0  prefixlen 64  scopeid 0x20<link>
        unspec 9E-6E-87-A5-EB-60-84-D0-00-00-00-00-00-00-00-00  txqueuelen 1000  (UNSPEC)
        RX packets 2  bytes 132 (132.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 95  bytes 9480 (9.2 KiB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

wlan0: flags=4099<UP,BROADCAST,MULTICAST>  mtu 1500
        ether b8:27:eb:3c:0e:b2  txqueuelen 1000  (Ethernet)
        RX packets 0  bytes 0 (0.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 0  bytes 0 (0.0 B)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

wpan0: flags=195<UP,BROADCAST,RUNNING,NOARP>  mtu 123
        unspec 9E-6E-87-A5-EB-60-84-D0-00-00-00-00-00-00-00-00  txqueuelen 300  (UNSPEC)
        RX packets 61  bytes 4775 (4.6 KiB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 107  bytes 8199 (8.0 KiB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```

The part which is important for us, is the next fragment:

``` bash
lowpan0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1280
        inet6 fe80::a9cd:ff:fe00:4204  prefixlen 64  scopeid 0x20<link>
        inet6 fe80::9c6e:87a5:eb60:84d0  prefixlen 64  scopeid 0x20<link>
        unspec 9E-6E-87-A5-EB-60-84-D0-00-00-00-00-00-00-00-00  txqueuelen 1000  (UNSPEC)
        RX packets 2  bytes 132 (132.0 B)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 95  bytes 9480 (9.2 KiB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0
```

As you can see there is two IP in this network interface. But you need to remember the second one. We will use later with NuttX.
`fe80::9c6e:87a5:eb60:84d0``

Go to the place where lives the repo that you downloaded previously. Go to ``micro-ROS-bridge_RPI/RPI_6lowpan/Examples/6lowpan_recv``
Finally, execute `recv_demo` telling to open the `61616` port. Type this command: ``./recv_demo 61616``

At this point, the RPI is ready to receive incoming packages.

### NuttX part:

Execute ``udp_6lowpan`` application.
This will ask you if you want to configure the network. Type N, because is already configured.
Now you're in the main menu of the App. Type ``write`` to start the sending mode and will ask for the next data:
- The destination IP: This is the IP of the RPI which we saved previously.
- The destination port: For this specific example is 61617.
- The origin port: This is the port that we want to open to send the message, for example, we could use 61618.

At this point it should look like this:

``` bash
nsh> udp_6lowpan                                                                
Do you want to execute the automatic WPAN configuration? (y/n)                  

Available commands                                                              
 -To send a package type: write                                                 
 -To receive a package type: read                                               
 -To exit type: quit                                                            
Introduce the IVP6 Destination                                                  
Introduce the port destination                                                  
Introduce the port origin                                                       
Conection data:                                                                 
 -Dest_IP: fe80::857:adfe:5a82:c7ac                                             
 -Dest_Port: 61616                                                              
 -Origin_Port: 61617                                                            
Introduce a message to send:
```
Now if you type somenthing and press enter NuttX should show the next:
```bash
Introduce a message to send:                                                    
Sending 5 characters: hello  
```

And the RPI, should show the next message:
```bash
Received (from fe80::2be:adde:de:fa00): hello
```

## Sending a message from the Raspbian to NuttX:

**Very important note! :**
The Linux 6lowpan utility needs to have a ping response before sending a data package, but the ping implementation of NuttX is not compatible. So the solution for this problem is to send a message from NuttX to Linux, just to add the NuttX direction to the neighborhood table.

If this does not work, do as follow:

Check if the Nuttx address is part within the Linux/Raspbian neighborhood table.
If it is saved there, remove it:
```bash
 $ ip neigh  # a bunch of address shall appear including yours (if you send a
	message already)
   fe80::2be:adde:de:fa00 dev lowpan0  FAILED
 $ sudo ip neigh delete fe80::2be:adde:de:fa00 dev lowpan0 # Remove it.
```

Then, once deleted, add the Nuttx device it permanently (until reboot):

```bash
 $ sudo ip neigh add fe80::2be:adde:de:fa00 dev lowpan0 00:be:ad:de:00:de:fa:00 # Add it with the
corret  Hardware address.
```

### NuttX part:
First, we need the ip of the board, so type ``ifconfig`` in the main menu. This should return somenthing like this:
```bash
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

             IPv6   TCP   UDP  ICMPv6                                           
Received     0000  0000  0000  0000                                             
Dropped      0000  0000  0000  0000                                             
  IPv6        VHL: 0000                                                         
  Checksum   ----  0000  0000  ----                                             
  TCP         ACK: 0000   SYN: 0000                                             
              RST: 0000  0000                                                   
  Type       0000  ----  ----  0000                                             
Sent         0000  0000  0000  0000                                             
  Rexmit     ----  0000  ----  ----    
```

For us is important to remember the **inet6_addr** which is: ``fe80::2be:adde:de:fa00``

Execute ``udp_6lowpan`` app, type ``N`` to the configuration request and finally type `read`, to start the receiving mode.
Type the port that you want to open, for example, the 61616. And it should look like this mean is waiting for the incoming message.

```bash
Available commands                                                              
 -To send a package type: write                                                 
 -To receive a package type: read                                               
 -To exit type: quit                                                            
Introduce the reception port                                                    
Listening on 61616 for input packets   
```

### Raspbian Part:

Go to the places that lives the previous download repo. Then go to this folder: ``/micro-ROS-bridge_RPI/RPI_6lowpan/Examples/6lowpan_send``

And execute with root privileges the next app: ``send_demo``.
These demos have two arguments: The first is the port to open in the destination and the second is the destination IP. An example of this specific demo could be:
``sudo send_demo 61616 fe80::2be:adde:de:fa00``

This app returns any data.

On the other hand, in the NuttX board we will see the next:
```bash
Received 12 bytes from 80fe:0000:0000:0000:cda9:ff00:00fe:0442 port 39464       
Received packet: Hola mundo !
```
