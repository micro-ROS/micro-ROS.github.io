---
title: Nodes
permalink: /docs/tutorials/programming_rcl_rclc/node/
---

ROS 2 nodes are the ground element on ROS2 ecosystem. They will contain communicate between each other using publishers, subscriptions, services, ... .Further information about ROS 2 nodes can be found [here](https://docs.ros.org/en/galactic/Tutorials/Understanding-ROS2-Nodes.html)

// TODO: explain general micro-ROS initialization (allocator and support)
## <a name="init_node"/>Initialization

- Create a node with default configuration:
  ```C
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

  Node configuration will also be applied to its future elements (Publishers, subscribers, services, ...).

  // TODO: explain possible options and their meaning

  The API used to customize the node options differs between ROS2 distributions:

  Foxy: The `rcl_node_options_t` is used to configure the node

  ```C
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

  ```C
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

### <a name="node_end"/>Cleaning Up

To destroy a initialized node all entities owned by the node (Publishers, subscribers, services, ...) needs to be destroyed before the node itself:

```C
// Destroy created entities
...

// Destroy a node
rcl_node_fini(&node);
```

This will delete any automatically created infrastructure on the agent (if possible) and deallocate used memory on the client side.
