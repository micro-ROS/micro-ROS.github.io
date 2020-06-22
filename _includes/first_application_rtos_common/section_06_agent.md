---

### Olimex STM32-E407 Serial connection

Olimex development board is connected to the computer using the usb to serial cable:

<img width="400" style="padding-right: 25px;" src="../imgs/5.jpg">

***TIP:** Color codes are applicable to [this cable](https://www.olimex.com/Products/Components/Cables/USB-Serial-Cable/USB-Serial-Cable-F/). Make sure to match Olimex Rx with Cable Tx and vice-versa. Remember GND!*

---

### Olimex STM32-E407 USB connection

Olimex development board is connected to the computer using the USB OTG 2 connector (the miniUSB connector that is furthest from the Ethernet port).

---

Then run the agent:

```bash
# Run a micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev [device]
```

***TIP:** you can use this command to find your serial device name: `ls /dev/serial/by-id/*`*

## Test the sample micro-ROS app behaviour

Once the micro-ROS app is built and flashed, and the board is connected to a micro-ROS agent, let's check that everything is working in a new command line. 
We are going to listen to ping topic to check whether the Ping Pong node is publishing its own ping messages:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Subscribe to micro-ROS ping topic
ros2 topic echo /microROS/ping
```

You should see the topic messages published by the Ping Pong node every 5 seconds:

```
user@user:~$ ros2 topic echo /microROS/ping
stamp:
  sec: 20
  nanosec: 867000000
frame_id: '1344887256_1085377743'
---
stamp:
  sec: 25
  nanosec: 942000000
frame_id: '730417256_1085377743'
---
```

On another command line, let's subscribe to the pong topic

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Subscribe to micro-ROS pong topic
ros2 topic echo /microROS/pong
```

At this point, we know that our app is publishing pings. Let's check if it also answers to someone else pings in a new command line:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Send a fake ping
ros2 topic pub --once /microROS/ping std_msgs/msg/Header '{frame_id: "fake_ping"}'
```

Now, we should see on the ping subscriber our fake ping along with the board pings:

```
user@user:~$ ros2 topic echo /microROS/ping
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
stamp:
  sec: 305
  nanosec: 973000000
frame_id: '451230256_1085377743'
---
stamp:
  sec: 310
  nanosec: 957000000
frame_id: '2084670932_1085377743'
---
```

And in the pong subscriber, we should see the board's answer to our fake ping:

```
user@user:~$ ros2 topic echo /microROS/pong
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
```

### Multiple Ping Pong nodes

If you have multiple boards, by connecting them to the same ROS 2 space it is possible to see them interacting. In the case you only have one board, it is possible to see your micro-ROS ping pong app running on hardware interacting with the [ping pong app from the Linux tutorial](../../first_application_linux). When multiple ping pong nodes coexists, it is possible to see their output like this micro-ROS for Linux app:

```
UDP mode => ip: 127.0.0.1 - port: 8888
Ping send seq 1711620172_1742614911                      <---- This micro-ROS node sends a ping with ping ID "1711620172" and node ID "1742614911"
Pong for seq 1711620172_1742614911 (1)                   <---- The first mate pongs my ping 
Pong for seq 1711620172_1742614911 (2)                   <---- The second mate pongs my ping 
Pong for seq 1711620172_1742614911 (3)                   <---- The third mate pongs my ping 
Ping received with seq 1845948271_546591567. Answering.  <---- A ping is received from a mate identified as "546591567", let's pong it.
Ping received with seq 232977719_1681483056. Answering.  <---- A ping is received from a mate identified as "1681483056", let's pong it.
Ping received with seq 1134264528_1107823050. Answering. <---- A ping is received from a mate identified as "1107823050", let's pong it.
Ping send seq 324239260_1742614911
Pong for seq 324239260_1742614911 (1)
Pong for seq 324239260_1742614911 (2)
Pong for seq 324239260_1742614911 (3)
Ping received with seq 1435780593_546591567. Answering.
Ping received with seq 2034268578_1681483056. Answering.
```