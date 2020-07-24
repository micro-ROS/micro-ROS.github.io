## Testing the micro-ROS app

At this point, the micro-ROS app is built and flashed and the board is connected to a micro-ROS agent.
We now want to check that everything is working.

Open a new command line. We are going to listen to the `ping` topic
with ROS 2 to check whether the micro-ROS Ping Pong node is correctly publishing the expected pings:

```bash
source /opt/ros/foxy/setup.bash

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

At this point, we know that our micro-ROS app is publishing pings.
Let's check if it also answers to someone else's pings. If this works, it'll publish a pong.

So, first of all, let's subscribe with ROS 2 to the `pong` topic from a new shell
(notice that initially we don't expect to receive any pong, since none has been sent yet):

```bash
source /opt/ros/foxy/setup.bash

# Subscribe to micro-ROS pong topic
ros2 topic echo /microROS/pong
```

And now, let's publish a `fake_ping` with ROS 2 from yet another command line:

```bash
source /opt/ros/foxy/setup.bash

# Send a fake ping
ros2 topic pub --once /microROS/ping std_msgs/msg/Header '{frame_id: "fake_ping"}'
```

Now, we should see this `fake_ping` in the `ping` subscriber console,
along with the board's pings:

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

Also, we expect that, because of having received the `fake_ping`, the micro-ROS `pong` publisher will answer with a
`pong`. As a consequence, in the `pong` subscriber console,
we should see the board's answer to our `fake_ping`:

```
user@user:~$ ros2 topic echo /microROS/pong
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
```


### Multiple Ping Pong nodes

If you have multiple boards, by connecting them to the same ROS 2 space it is possible to see them interacting.
In the case you only have one board, it is possible to see your micro-ROS ping pong app running on hardware interacting
with the [ping pong app from the Linux tutorial](../../first_application_linux). When multiple ping pong nodes coexists,
it is possible to see their output like this micro-ROS for Linux app:

```
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