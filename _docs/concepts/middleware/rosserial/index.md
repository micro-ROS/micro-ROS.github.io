---
title: Micro XRCE-DDS compared to rosserial 
permalink: /docs/concepts/middleware/rosserial/
---

In ROS, there is a package that stands out when we want to send ROS messages through serial communications.
This package is rosserial.

rosserial allows platforms based on microcontrollers to communicate with a regular computer communicating with ROS network on its behalf.
rosserial provides the protocol to set such communication, using a client-server architecture approach.
rosserial-clients serialise data into the serial link, then, the serialised data is received by the rosserial-server and forwarded to the conventional ROS network.
An analogous process is done to forward data from the ROS network towards the microcontroller.
This rosserial-server can be used either in C++ or Python; meanwhile, the rosserial-client has a set of available supported microprocessors.

This solution is commonly used to integrate hardware pieces within Robots using ROS.
Rosserial, in those cases, acts as a bridge between hardware communication protocols and a ROS network.

## Micro XRCE-DDS

One of the capabilities of Micro XRCE-DDS is the use of serial connection between a microcontroller and a DDS/ROS 2 capable computer.
Such a connection is possible thanks to the use of OMG's DDS-XRCE standard and a serial transport layer.
This solution follows the same client-server architecture as rosserial, which it is one of the most suitable approaches when we speak of microcontroller communications.

The libraries in charge of implementing this architecture are the Client and the Agent.
Clients generate entities within the Agent which will act on behalf of the Clients on a DDS network.

This usage, as you can see, resembles the one used by rosserial, but they have subtle differences in their implementations which we will expose in this article.

## Micro XRCE-DDS vs rosserial

Now that we have a basic understanding of rosserial and Micro XRCE-DDS we will provide a comparison between them.

### Micro XRCE-DDS over Serial Transport

Micro XRCE-DDS, when communicating over serial transports (it allows communications over other transports such as UDP, TCP ...), uses a serial protocol with a predefined format.
This format is explained in the following frame dissection:

```
0        8        16       24                40                 X                 X+16
+--------+--------+--------+--------+--------+--------//--------+--------+--------+
| FLAG   |  SADD  |  RADD  |       LEN       |     PAYLOAD      |       CRC       |
+--------+--------+--------+--------+--------+--------//--------|--------+--------+
```

* `FLAG`: synchronization flag (0xFF).
* `SADD`: source address.
* `RADD`: remote address.
* `LEN`: payload length without framing (2 bytes in little-endian).
* `PAYLOAD`: serialised message with XRCE headers.
* `CRC`: message CRC after stuffing.

This is the message that is serialised on the link layer.
This is the message going from an towards the microcontroller thanks to two different operations: publish and subscribe.

## rosserial

In contrast, this is the rosserial frame:

```
0       8       16              32              40      56               X             X+16
+-------+-------+-------+-------+-------+-------+-------+-------//-------+------+------+
| FLAG  | PROT  |      LEN      |      LCRC     | TOPID |     PAYLOAD    |     MCRC    |
+-------+-------+-------+-------+-------+-------+-------+-------//-------|------+------+
```

* `FLAG`: synchronization flag (0xFF).
* `PROT`: protocol version.
* `LEN`: payload length (2 bytes in little-endian).
* `LCRC`: length CRC.
* `TOPID`: topic ID.
* `PAYLOAD`: serialised message.
* `MCRC`: message CRC.

As you can see comparing with the Micro XRCE-DDS serial frame, it uses a completely different frame.

## Comparison

The following table, summarises the key aspects of both implmentations:

| | Micro XRCE-DDS serial | rosserial |
|:-|:-:|:-:|
| **API** | C/C++ | C++ |
| **integrity** | [HDLC](https://en.wikipedia.org/wiki/High-Level_Data_Link_Control) framing | none |
| **security** | [CRC-16-CCITT](https://en.wikipedia.org/wiki/Cyclic_redundancy_check) | vague CRC (% 256) |
| **memcopy** | uxr_stream ---> serial_buffer ---> hardware_buffer | serialization_buffer ---> hardware_buffer |
| **memory usage** | uxr_stream + (2 * aux-buffer[42 B]) | 2 * serialization_buffer |
| **message size limit** | uxr_stream size | hardware_buffer size |
| **reliability** | yes | no |
| **routing** | yes | no |
| **overhead** | 7 B + 12 B* | 9 B |
| **architecture** | client-server | client-server |

_* Overhead is divided between framing, 7 B and that added by DDS-XRCE protocol 12 B._

As exposed in this table, Micro XRCE-DDS serial increases the number of memory operations but reduces in a great way the memory required for serial communications as it does not require that the hardware buffer has the same size as the serialisation buffer.
This use of buffers reduces memory usage.
Also, Micro XRCE-DDS serial provides routing and reliability capabilities in contrast with rosserial.
The DDS-XRCE protocol embraces standards for some of its parts, like the usage of standard CRC or a standard framing.
