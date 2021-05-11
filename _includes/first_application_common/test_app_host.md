## Testing the micro-ROS app

Now, we want to check that everything is working.

Open a new command line. We are going to listen to the `ping` topic
with ROS 2 to check whether the micro-ROS Ping Pong node is correctly publishing the expected pings:

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

At this point, we know that our app is publishing pings.
Let's check if it also answers to someone else's pings. If this works, it'll publish a pong.

So, first of all, let's subscribe with ROS 2 to the `pong` topic from a new shell
(notice that initially we don't expect to receive any pong, since none has been sent yet):

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Subscribe to micro-ROS pong topic
ros2 topic echo /microROS/pong
```

And now, let's publish a `fake_ping` with ROS 2 from yet another command line:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash

# Send a fake ping
ros2 topic pub --once /microROS/ping std_msgs/msg/Header '{frame_id: "fake_ping"}'
```

Now, we should see this `fake_ping` in the `ping` subscriber console,
along with the micro-ROS pings:

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

Also, we expect that, because of having received the `fake_ping`, the micro-ROS node will answer with a `pong`:

```
user@user:~$ ros2 run micro_ros_demos_rcl ping_pong
Ping send seq 1706097268_1085377743
Ping send seq 181171802_1085377743
Ping send seq 1385567526_1085377743
Ping send seq 926583793_1085377743
Ping send seq 1831510138_1085377743
Ping received with seq fake_ping. Answering.
Ping send seq 1508705084_1085377743
Ping send seq 1702133625_1085377743
Ping send seq 176104820_1085377743
```

As a consequence, in the `pong` subscriber console,
we should see the micro-ROS app answer to our `fake_ping`:

```
user@user:~$ ros2 topic echo /microROS/pong
stamp:
  sec: 0
  nanosec: 0
frame_id: fake_ping
---
```