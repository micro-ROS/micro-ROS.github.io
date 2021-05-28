# micro-ROS Types Utilities

## Functions

|                | Name           |
| -------------- | -------------- |
| MICRO_ROS_UTILITIES_PUBLIC rosidl_runtime_c__String | **[micro_ros_utilities_type_info](#function-micro_ros_utilities_type_info)**(const rosidl_message_type_support_t * type_support)<br>Returns a string with the type instronspection data.  |
| MICRO_ROS_UTILITIES_PUBLIC size_t | **[micro_ros_utilities_get_dynamic_size](#function-micro_ros_utilities_get_dynamic_size)**(const rosidl_message_type_support_t * type_support, const micro_ros_utilities_memory_conf_t conf)<br>Returns the dynamic memory size that will be used for a type.  |
| MICRO_ROS_UTILITIES_PUBLIC size_t | **[micro_ros_utilities_get_static_size](#function-micro_ros_utilities_get_static_size)**(const rosidl_message_type_support_t * type_support, const micro_ros_utilities_memory_conf_t conf)<br>Returns the static memory size that will be used for a type.  |
| MICRO_ROS_UTILITIES_PUBLIC bool | **[micro_ros_utilities_create_message_memory](#function-micro_ros_utilities_create_message_memory)**(const rosidl_message_type_support_t * type_support, void * ros_msg, const micro_ros_utilities_memory_conf_t conf)<br>Allocates the dynamic memory required for a message.  |
| MICRO_ROS_UTILITIES_PUBLIC bool | **[micro_ros_utilities_create_static_message_memory](#function-micro_ros_utilities_create_static_message_memory)**(const rosidl_message_type_support_t * type_support, void * ros_msg, const micro_ros_utilities_memory_conf_t conf, uint8_t * buffer, size_t buffer_len)<br>Allocates the memory required for a message in a user-provided buffer.  |
| MICRO_ROS_UTILITIES_PUBLIC bool | **[micro_ros_utilities_destroy_message_memory](#function-micro_ros_utilities_destroy_message_memory)**(const rosidl_message_type_support_t * type_support, void * ros_msg, const micro_ros_utilities_memory_conf_t conf)<br>Deallocates the dynamic memory of a message.  |

## Attributes

|                | Name           |
| -------------- | -------------- |
| const micro_ros_utilities_memory_conf_t | **micro_ros_utilities_memory_conf_default**  |

## Types Documentation

### typedef micro_ros_utilities_memory_rule_t

```cpp
typedef struct micro_ros_utilities_memory_rule_t micro_ros_utilities_memory_rule_t;
```


### typedef micro_ros_utilities_memory_conf_t

```cpp
typedef struct micro_ros_utilities_memory_conf_t micro_ros_utilities_memory_conf_t;
```



## Functions Documentation

### function micro_ros_utilities_type_info

```cpp
MICRO_ROS_UTILITIES_PUBLIC rosidl_runtime_c__String micro_ros_utilities_type_info(
    const rosidl_message_type_support_t * type_support
)
```

Returns a string with the type instronspection data.

**Parameters**:

  * **type_support** ROS 2 typesupport


**Return**: `rosidl_runtime_c__String` string containing data



------------------


| Attribute  | Adherence   |
|  -------- | -------- |
| Allocates Memory  | Yes   |
| Thread-Safe  | No   |
| Uses Atomics  | No   |
| Lock-Free  | Yes   |


### function micro_ros_utilities_get_dynamic_size

```cpp
MICRO_ROS_UTILITIES_PUBLIC size_t micro_ros_utilities_get_dynamic_size(
    const rosidl_message_type_support_t * type_support,
    const micro_ros_utilities_memory_conf_t conf
)
```

Returns the dynamic memory size that will be used for a type.

**Parameters**:

  * **type_support** ROS 2 typesupport
  * **conf** Utils configurator


**Return**: `size_t` Size in Bytes that will be used



------------------


| Attribute  | Adherence   |
|  -------- | -------- |
| Allocates Memory  | Yes   |
| Thread-Safe  | No   |
| Uses Atomics  | No   |
| Lock-Free  | Yes   |


### function micro_ros_utilities_get_static_size

```cpp
MICRO_ROS_UTILITIES_PUBLIC size_t micro_ros_utilities_get_static_size(
    const rosidl_message_type_support_t * type_support,
    const micro_ros_utilities_memory_conf_t conf
)
```

Returns the static memory size that will be used for a type.

**Parameters**:

  * **type_support** ROS 2 typesupport
  * **conf** Utils configurator


**Return**: `size_t` Size in Bytes that will be used



------------------


| Attribute  | Adherence   |
|  -------- | -------- |
| Allocates Memory  | Yes   |
| Thread-Safe  | No   |
| Uses Atomics  | No   |
| Lock-Free  | Yes   |


### function micro_ros_utilities_create_message_memory

```cpp
MICRO_ROS_UTILITIES_PUBLIC bool micro_ros_utilities_create_message_memory(
    const rosidl_message_type_support_t * type_support,
    void * ros_msg,
    const micro_ros_utilities_memory_conf_t conf
)
```

Allocates the dynamic memory required for a message.

**Parameters**:

  * **type_support** ROS 2 typesupport
  * **ros_msg** ROS 2 msg with no type
  * **conf** Utils configurator


**Return**: `bool` true if success



------------------


| Attribute  | Adherence   |
|  -------- | -------- |
| Allocates Memory  | Yes   |
| Thread-Safe  | No   |
| Uses Atomics  | No   |
| Lock-Free  | Yes   |


### function micro_ros_utilities_create_static_message_memory

```cpp
MICRO_ROS_UTILITIES_PUBLIC bool micro_ros_utilities_create_static_message_memory(
    const rosidl_message_type_support_t * type_support,
    void * ros_msg,
    const micro_ros_utilities_memory_conf_t conf,
    uint8_t * buffer,
    size_t buffer_len
)
```

Allocates the memory required for a message in a user-provided buffer.

**Parameters**:

  * **type_support** ROS 2 typesupport
  * **ros_msg** ROS 2 msg with no type
  * **conf** Utils configurator
  * **buffer** User buffer
  * **buffer_len** User buffer length


**Return**: `bool` true if success



------------------


| Attribute  | Adherence   |
|  -------- | -------- |
| Allocates Memory  | Yes   |
| Thread-Safe  | No   |
| Uses Atomics  | No   |
| Lock-Free  | Yes   |


### function micro_ros_utilities_destroy_message_memory

```cpp
MICRO_ROS_UTILITIES_PUBLIC bool micro_ros_utilities_destroy_message_memory(
    const rosidl_message_type_support_t * type_support,
    void * ros_msg,
    const micro_ros_utilities_memory_conf_t conf
)
```

Deallocates the dynamic memory of a message.

**Parameters**:

  * **type_support** ROS 2 typesupport
  * **ros_msg** ROS 2 msg with no type
  * **conf** Utils configurator


**Return**: `bool` true if success



------------------


| Attribute  | Adherence   |
|  -------- | -------- |
| Allocates Memory  | Yes   |
| Thread-Safe  | No   |
| Uses Atomics  | No   |
| Lock-Free  | Yes   |



## Attributes Documentation

### variable micro_ros_utilities_memory_conf_default

```cpp
static const micro_ros_utilities_memory_conf_t micro_ros_utilities_memory_conf_default =
{20, 5, 5, NULL, 0, NULL};
```





-------------------------------