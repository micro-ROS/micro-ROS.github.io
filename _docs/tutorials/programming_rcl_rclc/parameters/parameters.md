---
title: Parameter server
permalink: /docs/tutorials/programming_rcl_rclc/parameters/
---

ROS 2 parameters allow the user to create variables on a node and manipulate/read them with different ROS2 commands. Further information about ROS 2 parameters can be found [here](https://docs.ros.org/en/galactic/Tutorials/Parameters/Understanding-ROS2-Parameters.html)

Ready to use code related to this tutorial can be found in [`rclc/rclc_examples/src/example_parameter_server.c`](https://github.com/ros2/rclc/blob/master/rclc_examples/src/example_parameter_server.c). Fragments of code from this example is used on this tutorial.

Note: micro-ROS parameter server is only supported on ROS2 galactic distribution

- [Initialization](#initialization)
- [Memory requirements](#memory-requirements)
- [Callback](#callback)
- [Add a parameter](#add-a-parameter)
- [Cleaning up](#cleaning-up)

## Initialization

- Default initialization:
    ```c
    // Parameter server object
    rclc_parameter_server_t param_server;

    // Initialize parameter server with default configuration
    rcl_ret_t rc = rclc_parameter_server_init_default(&param_server, &node);

    if (RCL_RET_OK != rc) {
    ... // Handle error
    return -1;
    }
    ```

- Custom options:

    The user can configure whether to notify parameter changes to the rest of nodes (`notify_changed_over_dds`) and the maximum number of parameters (`max_params`) allowed on the `rclc_parameter_server_t` object:

    ```c
    // Parameter server object
    rclc_parameter_server_t param_server;

    // Initialize parameter server options
    const rclc_parameter_options_t options = {
        .notify_changed_over_dds = true,
        .max_params = 4 };

    // Initialize parameter server with configured options
    rcl_ret_t rc = rclc_parameter_server_init_with_option(&param_server, &node, &options);

    if (RCL_RET_OK != rc) {
    ... // Handle error
    return -1;
    }
    ```

## Memory requirements
The parameter server uses 4 services and an optional publisher, this needs to be taken into account on the `rmw-microxredds` package memory configuration:

```json
# colcon.meta example with memory requirements to use a parameter server
{
    "names": {
        "rmw_microxrcedds": {
            "cmake-args": [
                "-DRMW_UXRCE_MAX_NODES=1",
                "-DRMW_UXRCE_MAX_PUBLISHERS=1",
                "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=0",
                "-DRMW_UXRCE_MAX_SERVICES=4",
                "-DRMW_UXRCE_MAX_CLIENTS=0"
            ]
        }
    }
}
```

On runtime, the variable `RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER` defines the RCLC executor handles required for a parameter server:

```c
// Executor init example with the minimum RCLC executor handles required
rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
rc = rclc_executor_init(
    &executor, &support.context,
    RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER, &allocator);
```

## Callback

When adding the parameter server to the executor, a callback can be configured.
This callback will be executed after a parameter value is modified.

A pointer to the changed parameter is passed as first and only argument. Example:
```c
void on_parameter_changed(Parameter * param)
{
    // Get parameter name
    printf("Parameter %s modified.", param->name.data);

    // Get parameter type
    switch (param->value.type)
    {
        // Get parameter value acording type
        case RCLC_PARAMETER_BOOL:
            printf(" New value: %d (bool)", param->value.bool_value);
            break;
        case RCLC_PARAMETER_INT:
            printf(" New value: %ld (int)", param->value.integer_value);
            break;
        case RCLC_PARAMETER_DOUBLE:
            printf(" New value: %f (double)", param->value.double_value);
            break;
        default:
            break;
    }

    printf("\n");
}
```
Once the parameter server and the executor are initialized, the parameter server must be added to the executor in order to accept parameters commands from ROS2:
```c
// Add parameter server to executor including defined callback
rc = rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed);
```

Note that this callback is optional as its just an event information for the user. To use the parameter server without a callback:
```c
// Add parameter server to executor without callback
rc = rclc_executor_add_parameter_server(&executor, &param_server, NULL);
```

## Add a parameter

micro-ROS parameter server supports the following parameter types:

- Boolean:
```c
const char * parameter_name = "parameter_bool";
bool param_value = true;

// Add parameter to the server
rcl_ret_t rc = rclc_add_parameter(&param_server, parameter_name, RCLC_PARAMETER_BOOL);

// Set parameter value (Triggers parameter change callback)
rc = rclc_parameter_set_bool(&param_server, parameter_name, param_value);

// Get parameter value on param_value
rc = rclc_parameter_get_bool(&param_server, "param1", &param_value);

if (RCL_RET_OK != rc) {
  ... // Handle error
  return -1;
}
```

- Integer:
```c
const char * parameter_name = "parameter_int";
int param_value = 100;

// Add parameter to the server
rcl_ret_t rc = rclc_add_parameter(&param_server, parameter_name, RCLC_PARAMETER_INT);

// Set parameter value
rc = rclc_parameter_set_int(&param_server, parameter_name, param_value);

// Get parameter value on param_value
rc = rclc_parameter_get_int(&param_server, parameter_name, &param_value);
```

- Double:
```c
const char * parameter_name = "parameter_double";
double param_value = 0.15;

// Add parameter to the server
rcl_ret_t rc = rclc_add_parameter(&param_server, parameter_name, RCLC_PARAMETER_DOUBLE);

// Set parameter value
rc = rclc_parameter_set_double(&param_server, parameter_name, param_value);

// Get parameter value on param_value
rc = rclc_parameter_get_double(&param_server, parameter_name, &param_value);
```

## Cleaning up

To destroy an initialized parameter server:

```c
// Delete parameter server
rclc_parameter_server_fini(&param_server, &node);
```

This will delete any automatically created infrastructure on the agent (if possible) and deallocate used memory on the client side.