---
title: Handling messages memory in micro-ROS
permalink: /docs/tutorials/advanced/handling_type_memory/
---

<img src="https://img.shields.io/badge/Applies_to-all_current_distros-green" style="display:inline"/>

This page aims to explain how to handle messages and types memory in micro-ROS.

First of all, since the micro-ROS user is in an embedded C99 environment, it is important to be aware of what messages and ROS 2 types are being used in order to handle memory correctly.

Two approaches are presented in this tutorial: manual memory allocation, and automated approach using the [`micro_ros_utilities`](https://github.com/micro-ROS/micro_ros_utilities) package:

- [Manual allocation](#manual-allocation)
  - [Sequence types in micro-ROS](#sequence-types-in-micro-ros)
  - [Compound types in micro-ROS](#compound-types-in-micro-ros)
  - [Sequences of compound types](#sequences-of-compound-types)
- [micro-ROS utilities](#micro-ros-utilities)

# Manual allocation

By watching the `.msg` or `.srv` of the types used in a micro-ROS application, you can determine the type of each member. Currently, the following types are supported:
- Basic type
- Array type
- Sequence type
- Compound type

Let's take an example `.mgs` for clarification:

```
# MyType.msg
std_msgs/Header header
int32[] values
float64 duration
int8[10] coefficients
string name
```

In this example:
- the member `duration` is a **basic type member**,.
- the member `values` is a **sequence type member** because it has a unbounded sequence of `int32`, in this case.
- the member `coefficients` is an **array type member** because it has a bounded sequence of 10 units of `int8`, in this case.
- the member `header` is an **compound type member** because it refers to complex type described in the same or other ROS 2 package.
- the member `name` is an **string type member** and should be understood as a `char[]` (sequence type member).

When dealing with the **micro-ROS typesupport** the developer needs to take into account how this message is going to be handled in the C99 API of micro-ROS. In general, the micro-ROS typesupport will create a C99 struct representation of the message:

```c
typedef struct mypackage__msg__MyType
{
  std_msgs__msg__Header header;
  rosidl_runtime_c__int32__Sequence values;
  double duration;
  int8 coefficients[10];
  rosidl_runtime_c__String name;  // equal to rosidl_runtime_c__char__Sequence
} mypackage__msg__MyType;
```

So when in an application has a variable of this type, for example `mypackage__msg__MyType mymsg;`, we know that:
- `mymsg.coefficients` has a C array of `int8`
- `mymsg.duration` is a `double` member

but, what happens with the `...Sequence` and the compound type member?

## Sequence types in micro-ROS

A **sequence type member** is an especial type member that hosts a pointer `data`, a `size` and a `capacity` value. The pointer should have memory for storing up to `capacity` values and `size` member shows how many element are currently in the sequence. Usually in micro-ROS, the user is in charge of assigning memory and values to this sequence members.

In the case of `MyType.msg`, the `values` sequence member is represented in C99 as this struct:

```c
typedef struct rosidl_runtime_c__int32__Sequence
{
  int32_t* data;    /* The pointer to an array of int32 */
  size_t size;      /* The number of valid items in data */
  size_t capacity;  /* The number of allocated items in data */
} rosidl_runtime_c__int32__Sequence;
```

So user need to handle the type like:

```c
mypackage__msg__MyType mymsg;

// mymsg.values.data is NULL or garbage now
// mymsg.values.size is 0 or garbage now
// mymsg.values.capacity is 0 or garbage now

// Assigning dynamic memory to the sequence
mymsg.values.capacity = 100;
mymsg.values.data = (int32_t*) malloc(mymsg.values.capacity * sizeof(int32_t));
mymsg.values.size = 0;

// Assigning static memory to the sequence
static int32_t memory[100];
mymsg.values.capacity = 100;
mymsg.values.data = memory;
mymsg.values.size = 0;

// Filling some data
for(int32_t i = 0; i < 3; i++){
  mymsg.values.data = i;
  mymsg.values.size++;
}
```

## Compound types in micro-ROS

When dealing with a compound type, the user should recursively inspect the types in order to determine how to handle each internal member.

For example in the `MyType.msg` example, the `header` member has the following structure:

```c
typedef struct std_msgs__msg__Header
{
  builtin_interfaces__msg__Time stamp;
  rosidl_runtime_c__String frame_id;
} std_msgs__msg__Header;
```

Remember that `rosidl_runtime_c__String` is equivalent to `rosidl_runtime_c__char__Sequence`. And `builtin_interfaces__msg__Time` looks like:

```c
typedef struct builtin_interfaces__msg__Time
{
  int32_t sec;
  uint32_t nanosec;
} builtin_interfaces__msg__Time;
```

To initialize the `header` member of `MyType.msg`:

```c
mypackage__msg__MyType mymsg;

// Assigning dynamic memory to the frame_id char sequence
mymsg.header.frame_id.capacity = 100;
mymsg.header.frame_id.data = (char*) malloc(mymsg.values.capacity * sizeof(char));
mymsg.header.frame_id.size = 0;

// Assigning value to the frame_id char sequence
strcpy(mymsg.header.frame_id.data, "Hello World");
mymsg.header.frame_id.size = strlen(mymsg.header.frame_id.data);

// Assigning value to other members
mymsg.stamp.sec = 10;
mymsg.stamp.nanosec = 20;
```

## Sequences of compound types

Users should take into account that **sequence type member** of **compound type member** are also valid ROS 2 type. For example, let's modify the previous example:

```
# MyComplexType.msg
std_msgs/Header[] multiheaders
int32[] values
float64 duration
int8[10] coefficients
string name
```

In this case, the generated typesupport will be:

```c
typedef struct mypackage__msg__MyComplexType
{
  std_msgs__msg__Header__Sequence multiheaders;
  rosidl_runtime_c__int32__Sequence values;
  double duration;
  int8 coefficients[10];
  rosidl_runtime_c__String name;  // equal to rosidl_runtime_c__char__Sequence
} mypackage__msg__MyComplexType;
```

Notice that `multiheaders` is a **sequence type member**, so it should be handled properly, but also it is a **compound type member** which needs to be handled recursively, initializing its own members. For example:

```c
mypackage__msg__MyComplexType mymsg;

// Init the multiheaders sequence
mymsg.multiheaders.capacity = 10;
mymsg.multiheaders.data = (std_msgs__msg__Header*) malloc(mymsg.values.capacity * sizeof(std_msgs__msg__Header));
mymsg.multiheaders.size = 0;

// Filling some data
for(int32_t i = 0; i < 3; i++){
  mymsg.values.data = i;

  // Add memory to this sequence element frame_id
  mymsg.multiheaders.data[i].frame_id.capacity = 100;
  mymsg.multiheaders.data[i].frame_id.data = (char*) malloc(mymsg.multiheaders.data[i].frame_id.capacity * sizeof(char));
  mymsg.multiheaders.data[i].frame_id.size = 0;

  // Assigning value to the frame_id char sequence
  strcpy(mymsg.multiheaders.data[i].frame_id.data, "Hello World");
  mymsg.multiheaders.data[i].frame_id.size = strlen(mymsg.multiheaders.data[i].frame_id.data);

  // Assigning value to other members
  mymsg.multiheaders.data[i].stamp.sec = 10;
  mymsg.multiheaders.data[i].stamp.nanosec = 20;

  mymsg.multiheaders.size++;
}
```

# micro-ROS utilities

Due to the inclusion of [`rosidl_typesupport_introspection_c`](https://github.com/ros2/rosidl/tree/rolling/rosidl_typesupport_introspection_c), an automated memory handling for micro-ROS types is available. The tools related to this feature are available in the package [`micro_ros_utilities`](https://github.com/micro-ROS/micro_ros_utilities).

The documentation of the package [`micro_ros_utilities`](https://github.com/micro-ROS/micro_ros_utilities) is available [here](https://micro.ros.org/docs/api/utils/).

This package is able to auto-assign memory to a certain message struct using default dynamic memory allocators, for example, using the previouly declated type:

```c
mypackage__msg__MyType mymsg;

static micro_ros_utilities_memory_conf_t conf = {0};

bool success = micro_ros_utilities_create_message_memory(
  ROSIDL_GET_MSG_TYPE_SUPPORT(mypackage, msg, MyType),
  &mymsg,
  conf
);
```

This code will init all the string and sequences recursively in `MyType` type. The size of this memory slots will be by default the ones in [`micro_ros_utilities_memory_conf_default`](https://github.com/micro-ROS/micro_ros_utilities/blob/c829971bd33ac1f14a94aa722476110b4b59eaf9/include/micro_ros_utilities/type_utilities.h#L51), that is:
- String will have 20 characters
- ROS 2 types sequences will have a length of 5
- Basic types sequences will have a length of 5

This defaults can be overriden using:

```c
mypackage__msg__MyType mymsg;

static micro_ros_utilities_memory_conf_t conf = {0};

conf.max_string_capacity = 50;
conf.max_ros2_type_sequence_capacity = 5;
conf.max_basic_type_sequence_capacity = 5;

bool success = micro_ros_utilities_create_message_memory(
  ROSIDL_GET_MSG_TYPE_SUPPORT(mypackage, msg, MyType),
  &mymsg,
  conf
);
```

To customize the length of each member of the struct, a complex rules approach can be used as in the following example:

```c
mypackage__msg__MyComplexType mymsg;

static micro_ros_utilities_memory_conf_t conf = {0};

micro_ros_utilities_memory_rule_t rules[] = {
  {"multiheaders", 4},
  {"multiheaders.frame_id", 60},
  {"name", 10}
};
conf.rules = rules;
conf.n_rules = sizeof(rules) / sizeof(rules[0]);

// member named "values" of MyComplexType will have the default max_basic_type_sequence_capacity

bool success = micro_ros_utilities_create_message_memory(
  ROSIDL_GET_MSG_TYPE_SUPPORT(mypackage, msg, MyComplexType),
  &mymsg,
  conf
);
```

Is also possible to use a user-provided buffer to allocate memory:

```c
mypackage__msg__MyComplexType mymsg;

static micro_ros_utilities_memory_conf_t conf = {0};

static uint8_t my_buffer[1000];

bool success = micro_ros_utilities_create_static_message_memory(
  ROSIDL_GET_MSG_TYPE_SUPPORT(mypackage, msg, MyComplexType),
  &mymsg,
  conf,
  my_buffer,
  sizeof(my_buffer)
);
```

The library provides utilies for calculating the size that both approaches will use with a certain configuration. Notice that this amount of memory is only the dynamic usage or the usage in the user provided buffer, `sizeof(mypackage__msg__MyComplexType)` is not taken into account.

```c
mypackage__msg__MyComplexType mymsg;

static micro_ros_utilities_memory_conf_t conf = {0};

size_t dynamic_size = micro_ros_utilities_get_dynamic_size(
    ROSIDL_GET_MSG_TYPE_SUPPORT(mypackage, msg, MyComplexType),
    conf
);

size_t static_size = micro_ros_utilities_get_static_size(
    ROSIDL_GET_MSG_TYPE_SUPPORT(mypackage, msg, MyComplexType),
    conf
);
```

Finally, a destruction function is also provided for messages allocated in dynamic memory:


```c
mypackage__msg__MyComplexType mymsg;

// Release memory previously allocated with micro_ros_utilities_create_message_memory

bool success = micro_ros_utilities_destroy_message_memory(
  ROSIDL_GET_MSG_TYPE_SUPPORT(mypackage, msg, MyComplexType),
  &mymsg,
  conf
);
```
