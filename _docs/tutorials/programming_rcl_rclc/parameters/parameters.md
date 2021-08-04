---
title: Parameters
permalink: /docs/tutorials/programming_rcl_rclc/parameters/
---

## <a name="parameters"/>Parameters
// TODO: add link to example

## <a name="parameters_server"/>Creating a parameter server
 

```C
// Parameter server object
rclc_parameter_server_t param_server;

// Initialize parameter server with default configuration
rcl_ret_t rc = rclc_parameter_server_init_default(&param_server, &node);

if (RCL_RET_OK != rc) {
  printf("Error creating parameter server\n");
  return -1;
}

// Configure executor with atleast RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER handles
rclc_executor_t executor;
rclc_executor_init(&executor, &support.context, RCLC_PARAMETER_EXECUTOR_HANDLES_NUMBER, &allocator);

// Add parameter server to executor
rc = rclc_executor_add_parameter_server(&executor, &param_server, NULL);
```

- Parameter changed callback

When adding the paramater server to the executor, a callback for parameter changes can be passed.
This callback will be called after a parameter value is modified.

A pointer to the changed parameter is passed as first and only argument. Example:
```C
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

// Add parameter server to executor including defined callback
rc = rclc_executor_add_parameter_server(&executor, &param_server, on_parameter_changed);
```

// TODO: explain options on creation
// TODO: explain destruction
// TODO: explain memory requirements

## <a name="parameters_add"/>Add a parameter

// TODO: improve explanation of types

- Bool parameter
```C
const char * parameter_name = "parameter_bool";
bool param_value = true;

// Add parameter to the server
rcl_ret_t rc = rclc_add_parameter(&param_server, parameter_name, RCLC_PARAMETER_BOOL);

// Set parameter value (Triggers parameter change callback)
rc = rclc_parameter_set_bool(&param_server, parameter_name, param_value);

// Get parameter value on param_value
rc = rclc_parameter_get_bool(&param_server, "param1", &param_value);

if (RCL_RET_OK != rc) {
  // Handle error
  return -1;
}
```

- Integer parameter
```C
const char * parameter_name = "parameter_int";
int param_value = 100;

// Add parameter to the server
rcl_ret_t rc = rclc_add_parameter(&param_server, parameter_name, RCLC_PARAMETER_INT);

// Set parameter value
rc = rclc_parameter_set_int(&param_server, parameter_name, param_value);

// Get parameter value on param_value
rc = rclc_parameter_get_int(&param_server, parameter_name, &param_value);
```

- Double parameter
```C
const char * parameter_name = "parameter_double";
double param_value = 0.15;

// Add parameter to the server
rcl_ret_t rc = rclc_add_parameter(&param_server, parameter_name, RCLC_PARAMETER_DOUBLE);

// Set parameter value
rc = rclc_parameter_set_double(&param_server, parameter_name, param_value);

// Get parameter value on param_value
rc = rclc_parameter_get_double(&param_server, parameter_name, &param_value);
```
