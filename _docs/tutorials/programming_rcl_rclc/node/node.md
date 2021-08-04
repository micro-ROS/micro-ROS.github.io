---
title: Nodes
permalink: /docs/tutorials/programming_rcl_rclc/node/
---

## <a name="node"/>Nodes

## <a name="init_node"/>Creating a node

// TODO: explain reliable vs best_effort

- Create a node:
```C
// TODO: explain allocator and support
rcl_allocator_t allocator = rcl_get_default_allocator();
rclc_support_t support;

rcl_ret_trc = rclc_support_init(&support, argc, argv, &allocator);
if (rc != RCL_RET_OK) {
  printf("Error creating support object\n");
  return -1;
}

// Node object
rcl_node_t node;
const char * node_name = "test_node";
const char * namespace = "test_namespace";
rc = rclc_node_init_default(&node, node_name, namespace, &support);
if (rc != RCL_RET_OK) {
  // Handle error
  printf("Error creating node\n");
  return -1;
}

```

## <a name="end_node"/>Destroy a node

```C
// Destroy a node
rc = rcl_node_fini(&node);

if (rc == RCL_RET_OK) {
  printf("Error destroying node\n");
}
```

## <a name="node_opt"/>Node options


```C
// TODO: explain allocator and support
rcl_allocator_t allocator = rcl_get_default_allocator();
rclc_support_t support;

// Node object
rcl_node_t node;
const char * node_name = "test_node";
const char * namespace = "test_namespace";

// TODO: explain options
rcl_node_options_t node_options = rcl_node_get_default_options();

rc = rclc_node_init_default(&node, node_name, namespace, &support, &node_options);
if (rc != RCL_RET_OK) {
  printf("Error creating node\n");
  return -1;
}

```