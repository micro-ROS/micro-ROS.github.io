---
title: Timeout RMW & QoS in RCLC APIs 
author: Maria Merlan
---
There are NEW APIs available to improve the usability and scope of micro-ROS:
 
- The new [RCLC QoS API](https://github.com/ros2/rclc/pull/119/) allows users to set custom QoS to better adjust the use cases. 
So far, users could only use default or best effort entities. Now, this API allows the user to set a whole rmw_qos_profile_t. All internal QoS available in RMW as well as the entities can be customized to adjust every use case.  
The RCLC QoS API applies to all entities. 
 
- The new [RMW timeout API](https://github.com/micro-ROS/rmw-microxrcedds/pull/153) helps the user to configure the publication time block time when using a reliable mode in micro-ROS.
When a topic is published in reliable mode in the RMW of Micro XRCE-DDS, the XRCE-DDS session will be run during a default time-out before sending an error, in order to confirm that the messages arrive to the Agent. This default timeout used to be managed by a buit-time configurable parameter. 
The new RMW API allows the user to set a specific value for the timeout parameter at run-time for each reliable publisher, meaning before publishing. 
 
This brings advantages:	
  - Users can control the blocking time of a reliable publication.
  - Users can reinforce time constraints to adjust the behaviour to their use case, allowing to fine-tune the timing of micro-ROS publisher operations.
 
These enhancements are in line with the strategy of improving usability and scope of micro-ROS. We thank the community and early adopters for their great insights that contribute to enhancing micro-ROS usability and to better focus real use cases. 
 
 
 
