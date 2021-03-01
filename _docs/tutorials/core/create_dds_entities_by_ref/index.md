---
title: How to use custom QoS in micro-ROS
permalink: /docs/tutorials/core/create_dds_entities_by_ref/
---


This tutorial explains the procedure for creating micro-ROS entities using fully configurable QoS settings. The micro-ROS default middleware (Micro XRCE-DDS Client) allows the user to take two different approaches for creating ROS 2 (DDS) entities in the micro-ROS Agent (Please check the [architecture section](https://micro-ros.github.io/docs/overview/) for detailed information):
- By XML (the default option in micro-ROS RMW)
- By reference

Using the *default option* the micro-ROS user will be able to create entities using RCLC functions such as `rclc_publisher_init_default` for reliable communications or `rclc_publisher_init_best_effort` for best effort communications. Please check [RCLC](https://github.com/ros2/rclc) for an updated list of convenience functions.

For those familiar with the QoS XML format in DDS (click [here](https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/core/policy/policy.html) for detailed information), the underlying QoS profile used by this default mode looks like this:

```xml
<!-- TOPIC -->
<dds>
  <topic>
      <name>[TOPIC NAME]</name>
      <dataType>[TOPIC TYPE]</dataType>
  </topic>
</dds>

<!-- DATA WRITER -->
<dds>
  <data_writer>
      <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
      <qos>
        <reliability>
            <kind>[WRITER RELIABILITY]</kind>
        </reliability>
      </qos>
      <topic>
        <kind>NO_KEY</kind>
        <name>[WRITER NAME]</name>
        <dataType>[WRITER TYPE]</dataType>
        <historyQos>
            <kind>KEEP_ALL</kind>
        </historyQos>
      </topic>
  </data_writer>
</dds>

<!-- DATA READER -->
<dds>
  <data_reader>
      <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
      <qos>
        <reliability>
            <kind>[READER RELIABILITY]</kind>
        </reliability>
      </qos>
      <topic>
        <kind>NO_KEY</kind>
        <name>[READER NAME]</name>
        <dataType>[READER TYPE]</dataType>
        <historyQos>
            <kind>KEEP_ALL</kind>
        </historyQos>
      </topic>
  </data_reader>
</dds>

```

But these QoS configurations may not fit some user's requirements. For these cases, micro-ROS allows the users to write their custom XML QoS and run the agent with a predefined set of QoS. Each entity will have its own label and the micro-ROS client will create the entities using just this reference.

Additionally, using references will also reduce the memory consumption of the micro-ROS client inside the MCU. This is because the parts of the code where XML are handled are just not build with the references approach.

Let's see how to create a micro-ROS node that creates entities with custom QoS. First of all, independently of which RTOS you have selected in [First micro-ROS Application on an RTOS
](https://micro-ros.github.io/docs/tutorials/core/first_application_rtos/) tutorial, you should have an app configuration file named `app-colcon.meta`.

Inside this `app-colcon.meta` file we can set application specific CMake options for the micro-ROS packages that are going to be crosscompiled. So, let's setup the `rmw_microxrcedds` in order to use references; your `app-colcon.meta` should look like this:

```
{
    "names": {
        "rmw_microxrcedds": {
            "cmake-args": [
                ...
                "-DRMW_UXRCE_CREATION_MODE=refs"
                ...
            ]
        }
    }
}
```

Of course you can combine these configurations with others, e.g. the ones described in the [Middleware Configuration](https://micro-ros.github.io/docs/tutorials/core/microxrcedds_rmw_configuration/) tutorial.

Once you have this parameter, write your micro-ROS application using RCLC default convenience functions. Just remember that now you are not providing the topic name but a "QoS reference label":

```c
#include <std_msgs/msg/int32.h>

...

std_msgs__msg__Int32 msg;

msg.data = 42;

...

rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "my_qos_label");
rcl_publish(&publisher, &msg, NULL);
 
...
```

This code will tell the micro-ROS Agent to create a publisher using just a text label: `my_qos_label`. This implies that the micro-ROS Agent must have a file where these labels are written along with some QoS profiles. Let's create a `custom_qos.refs` with the following content:

```xml
<profiles>
    <participant profile_name="participant_profile">
        <rtps>
            <name>default_xrce_participant</name>
        </rtps>
    </participant>

    <topic profile_name="my_qos_label__t">
        <name>rt/my_topic_name</name>
        <dataType>std_msgs::msg::dds_::Int32_</dataType>
        <historyQos>
          <kind>KEEP_LAST</kind>
          <depth>20</depth>
        </historyQos>
    </topic>

    <data_writer profile_name="my_qos_label__dw">
        <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
        <qos>
          <reliability>
            <kind>RELIABLE</kind>
          </reliability>
        </qos>
        <topic>
            <kind>NO_KEY</kind>
            <name>rt/my_topic_name</name>
            <dataType>std_msgs::msg::dds_::Int32_</dataType>
            <historyQos>
                <kind>KEEP_LAST</kind>
                <depth>20</depth>
            </historyQos>
        </topic>
    </data_writer>

</profiles>
```

When writting this XML file, special care about ROS2 to DDS name mangling has to be taken into account. Read more information about this [here](http://design.ros2.org/articles/topic_and_service_names.html#mapping-of-ros-2-topic-and-service-names-to-dds-concepts).

Once you have this file ready, just run the micro-ROS agent with the `-r` parameter:

```
ros2 run micro_ros_agent micro_ros_agent [PARAMETERS] -r custom_qos.refs
```

Once the entities are created and the topic is being published, you can check the QoS using:

```
$ ros2 topic info /std_msgs_msg_Int32 --verbose
Type: std_msgs/msg/Int32

Publisher count: 1

Node name: _CREATED_BY_BARE_DDS_APP_
Node namespace: _CREATED_BY_BARE_DDS_APP_
Topic type: std_msgs/msg/Int32
Endpoint type: PUBLISHER
GID: 01.0f.0b.5c.8b.7d.00.00.01.00.00.00.00.00.01.03.00.00.00.00.00.00.00.00
QoS profile:
  Reliability: RMW_QOS_POLICY_RELIABILITY_RELIABLE
  Durability: RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
  Lifespan: 2147483651294967295 nanoseconds
  Deadline: 2147483651294967295 nanoseconds
  Liveliness: RMW_QOS_POLICY_LIVELINESS_AUTOMATIC
  Liveliness lease duration: 2147483651294967295 nanoseconds

Subscription count: 0
```
