---
title: Handling messages memory in micro-ROS
permalink: /docs/tutorials/advanced/handling_type_memory/
---

This page aims to explain how to handle messages and types memory in micro-ROS.

First of all, since the micro-ROS user is in a embedded C99 environment, it is important to be aware of what messages and ROS 2 types are being used in order to handle memory correctly.

By watching the `.msg` or `.srv` of the types used in a micro-ROS application, you can determine if your type's members are a:
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

When dealing with the **micro-ROS typesupport** the application coder need to take into account how this message is going to be handled in the C99 API of micro-ROS. In general, the micro-ROS typesupport will create a C99 representation of this type with this struct:

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

When dealing with a type that uses a compound type, the user should recursively inspect the types in order to determine how to handle the each internal member. 

For example in the `MyType.msg` example, when we inspect the `header` member, it looks like:

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

So for example in order to init the `header` member of `MyType.msg`, an user should do:

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

User should take into account that **sequence type member** of **compound type member** are also valid ROS 2 type. For example, let's modify the previous example:

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

Notice that `multiheaders` is a **sequence type member**, so it should be handled properly, but also it is a **compound type member** so it should be handled recursively. For example:

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

