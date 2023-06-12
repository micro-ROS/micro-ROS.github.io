---
title: Nodes
permalink: /docs/tutorials/programming_rcl_rclc/node/
---

<img src="https://img.shields.io/badge/Written_for-Humble-green" style="display:inline"/> <img src="https://img.shields.io/badge/Tested_on-Rolling-green" style="display:inline"/> <img src="https://img.shields.io/badge/Tested_on-Iron-green" style="display:inline"/>

ROS 2 nodes are the main participants on ROS 2 ecosystem. They will communicate between each other using publishers, subscriptions, services, etc. Further information about ROS 2 nodes can be found [here](https://docs.ros.org/en/iron/Tutorials/Understanding-ROS2-Nodes.html)


- [Initialization](#initialization)
  - [Cleaning Up](#cleaning-up)
- [Lifecycle](#lifecycle)
  - [Initialization](#initialization-1)
  - [Callbacks](#callbacks)
  - [Running](#running)
  - [Cleaning Up](#cleaning-up-1)
  - [Limitations](#limitations)

## Initialization

- Create a node with default configuration:
  ```c
  // Initialize micro-ROS allocator
  rcl_allocator_t allocator = rcl_get_default_allocator();

  // Initialize support object
  rclc_support_t support;
  rcl_ret_t rc = rclc_support_init(&support, argc, argv, &allocator);

  // Create node object
  rcl_node_t node;
  const char * node_name = "test_node";

  // Node namespace (Can remain empty "")
  const char * namespace = "test_namespace";

  // Init default node
  rc = rclc_node_init_default(&node, node_name, namespace, &support);
  if (rc != RCL_RET_OK) {
    ... // Handle error
    return -1;
  }
  ```

- Create a node with custom options:

  The configuration of the node will also be applied to its future elements (Publishers, subscribers, services, ...). The node options are configured on the `rclc_support_t` object with a custom API:

  ```c
  // Initialize micro-ROS allocator
  rcl_allocator_t allocator = rcl_get_default_allocator();

  // Initialize and modify options (Set DOMAIN ID to 10)
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  rcl_init_options_init(&init_options, allocator);
  rcl_init_options_set_domain_id(&init_options, 10);

  // Initialize rclc support object with custom options
  rclc_support_t support;
  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

  // Create node object
  rcl_node_t node;
  const char * node_name = "test_node";

  // Node namespace (Can remain empty "")
  const char * namespace = "test_namespace";

  // Init node with configured support object
  rclc_node_init_default(&node, node_name, namespace, &support);

  if (rc != RCL_RET_OK) {
    ... // Handle error
    return -1;
  }
  ```

### Cleaning Up

To destroy a initialized node all entities owned by the node (Publishers, subscribers, services, ...) have to be destroyed before the node itself:

```c
// Destroy created entities (Example)
rcl_publisher_fini(&publisher, &node);
...

// Destroy the node
rcl_node_fini(&node);
```

This will delete the node from ROS2 graph, including any generated infrastructure on the agent (if possible) and used memory on the client.

## Lifecycle

The rclc lifecycle package provides convenience functions in C to bundle an rcl node with the ROS 2 Node Lifecycle state machine, similar to the [rclcpp Lifecycle Node](https://github.com/ros2/rclcpp/blob/master/rclcpp_lifecycle/include/rclcpp_lifecycle/lifecycle_node.hpp) for C++. Further information about ROS 2 node lifecycle can be found [here](https://design.ros2.org/articles/node_lifecycle.html)

An usage example is given in the [rclc_examples](https://github.com/ros2/rclc/blob/master/rclc_examples/src/example_lifecycle_node.c) package.

### Initialization

Creation of a lifecycle node as a bundle of an rcl node and the rcl lifecycle state machine. Assuming an already initialized node and executor:

```c
// Create rcl state machine
rcl_lifecycle_state_machine_t state_machine =
rcl_lifecycle_get_zero_initialized_state_machine();

// Create the lifecycle node
rclc_lifecycle_node_t my_lifecycle_node;
rcl_ret_t rc = rclc_make_node_a_lifecycle_node(
  &my_lifecycle_node,
  &node,
  &state_machine,
  &allocator);

// Register lifecycle services on the allocator
rclc_lifecycle_add_get_state_service(&lifecycle_node, &executor);
rclc_lifecycle_add_get_available_states_service(&lifecycle_node, &executor);
rclc_lifecycle_add_change_state_service(&lifecycle_node, &executor);
```

*Note: Executor needsto be equipped with 1 handle per node and per service*

### Callbacks

Optional callbacks are supported to act on lifecycle state changes. Example:

```c
rcl_ret_t my_on_configure() {
  printf("  >>> my_lifecycle_node: on_configure() callback called.\n");
  return RCL_RET_OK;
}
```

To add them to the lifecycle node:

```c
// Register lifecycle service callbacks
rclc_lifecycle_register_on_configure(&lifecycle_node, &my_on_configure);
rclc_lifecycle_register_on_activate(&lifecycle_node, &my_on_activate);
rclc_lifecycle_register_on_deactivate(&lifecycle_node, &my_on_deactivate);
rclc_lifecycle_register_on_cleanup(&lifecycle_node, &my_on_cleanup);
```

### Running

To change states of the lifecycle node:

```c
bool publish_transition = true;
rc += rclc_lifecycle_change_state(
  &my_lifecycle_node,
  lifecycle_msgs__msg__Transition__TRANSITION_CONFIGURE,
  publish_transition);

rc += rclc_lifecycle_change_state(
  &my_lifecycle_node,
  lifecycle_msgs__msg__Transition__TRANSITION_ACTIVATE,
  publish_transition);
```

Except for error processing transitions, transitions are usually triggered from outside, e.g., by ROS 2 services.

### Cleaning Up

To clean everything up, simply do

```c
rc += rcl_lifecycle_node_fini(&my_lifecycle_node, &allocator);
```

### Limitations

Lifecycle services cannot yet be called via ros2 lifecycle client (`ros2 lifecycle set /node ...`). Instead use the ros2 service CLI, (Example: `ros2 service call /node/change_state lifecycle_msgs/ChangeState "{transition: {id: 1, label: configure}}"`).
