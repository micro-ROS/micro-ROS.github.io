---
title: Nodes
permalink: /docs/tutorials/programming_rcl_rclc/node/
---

ROS 2 nodes are the main participants on ROS 2 ecosystem. They will communicate between each other using publishers, subscriptions, services, etc. Further information about ROS 2 nodes can be found [here](https://docs.ros.org/en/galactic/Tutorials/Understanding-ROS2-Nodes.html)

// TODO: explain general micro-ROS initialization (allocator and support). Where?

- [Initialization](#initialization)
  - [Cleaning Up](#cleaning-up)

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

  // TODO: explain possible options

  The configuration of the node will also be applied to its future elements (Publishers, subscribers, services, ...).The API used to customize the node options differs between ROS2 distributions:

  Foxy: The `rcl_node_options_t` is used to configure the node

  ```c
  // Initialize allocator and support objects
  ...

  // Create node object
  rcl_node_t node;
  const char * node_name = "test_node";

  // Node namespace (Can remain empty "")
  const char * namespace = "test_namespace";

  // Get default node options and modify them
  rcl_node_options_t node_ops = rcl_node_get_default_options();

  // Set node ROS domain ID to 10
  node_ops.domain_id = (size_t)(10);

  // Init node with custom options
  rc = rclc_node_init_with_options(&node, node_name, namespace, &support, &node_ops);

  if (rc != RCL_RET_OK) {
    ... // Handle error
    return -1;
  }
  ```

  Galactic: In this case, the node options are configured on the `rclc_support_t` object with a custom API

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
