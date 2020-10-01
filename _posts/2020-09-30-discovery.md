---
title: A brand new discovery mechanism in micro-ROS
author: francesca-finocchiaro
---

micro-ROS is now implementing client/server handshaking by discovery!

The discovery mechanism allows micro-ROS Clients to discover Agents in the network by UDP. If a Client doesn’t know of any Agent beforehand, it sends a discovery call by multicast and the reachable Agents respond by sending information about them, such as their IP and port. The Client matches automatically with the first Agent that provides information, and the communication between the matched entities proceeds in the usual way.

<img alt="Discovery" src="/img/posts/discovery.png" width="60%"/>

micro-ROS inherits its discovery mechanism from the [Micro XRCE-DDS library](https://micro-xrce-dds.docs.eprosima.com/en/latest/client.html#discovery-profile), which is then implemented in the RMW layer. Discovery is accessible by default at the Client level when using UDP transport, and can be activated in the Agent by using the `-d` flag when running it from terminal with the [snap command](https://discourse.ros.org/t/micro-xrce-dds-agent-now-on-snap/16523):
```
micro-xrce-dds-agent udp4 <args> -d
```
If you’re running instead the daemonized version of the Agent via the provided snap service, you can turn on the discovery mechanism like this:
```
snap set micro-xrce-dds-agent discovery="true"
```
Find [here](https://github.com/micro-ROS/micro-ROS-demos/tree/foxy/rclc/autodiscover_agent) an example showing how to use this new feature!
