---
title: Embedded rcutils analysis
permalink: /docs/concepts/client_library/rcutils_analysis/
---

## Proposed changes

The main issues found when porting rcutils package to embedded systems with RTOS during micro*ROS development were:
 * the lack of filesystem
 * dynamic memory allocations out of the RTOS control (usually with standard library implementations of `malloc` and `free`)
 * the POSIX API dependency

This document addresses the first two items.

To avoid the **filesystem** dependency without modifying high*level layers (such as `rcl` or `rmw`), it is proposed to use a combination between the available `rcl_logging_noop` package and a modified version of the filesystem dependencies in `rcutils`. This approach is enabled using the `RCUTILS_NO_FILESYSTEM` flag in compilation time. 

**Dynamic memory allocation** must be bounded to ROS2 custom allocators. `rcl` and `rmw` layers usually rely on `rcutils_get_default_allocator` function to obtain allocators, so a setter function is proposed.
This way, using `rcutils_set_default_allocator` function is possible to modify the defaults easing the use of custom memory management in embedded systems.

Dynamic memory allocation is also found when dealing with error handling in `rcutils`. By using `RCUTILS_AVOID_DYNAMIC_ALLOCATION` some string related `memmove` functions along the error handling are avoided.

## Dependencies in `rcl`, `rcutils` and `rmw`

*Test folders are excluded.*

### error_handling.h

* Macro **RCUTILS_SAFE_FWRITE_TO_STDERR** in `rcutils` error_handling.h:
[Check here](https://github.com/micro-ROS/rcutils/commit/bcaa00a6ed12fc62d05dc5e44521a1648fd2d07f#diff-1b06d4a1ccca0f0dff66d961923143a1L42)
  * Problem: use `fwrite` to print to stderr and relies on a filesytem
  * Appears in:
    * rcl/rcl/src/rcl/expand_topic_name.c
    * rcl/rcl/src/rcl/logging_rosout.c
    * rcl/rcl/src/rcl/logging.c
    * rcl/rcl/src/rcl/context.c
    * rcl/rcl_yaml_param_parser/src/parser.c
    * rcutils/src/split.c
    * rcutils/src/error_handling.c
    * rcutils/src/allocator.c
    * rcutils/src/logging.c
    * rcutils/src/error_handling_helpers.h
    * rcutils/include/rcutils/logging.h
    * rcutils/include/rcutils/error_handling.h
    * rcl/rcl/src/rcl/expand_topic_name.c
    * rcl/rcl/src/rcl/logging_rosout.c
    * rcl/rcl/src/rcl/logging.c
    * rcl/rcl/src/rcl/context.c
    * rcl/rcl_yaml_param_parser/src/parser.c
    * rmw/rmw/include/rmw/error_handling.h

* Macro **RCUTILS_SET_ERROR_MSG** in `rcutils` error_handling.h: [Check here](https://github.com/micro-ROS/rcutils/commit/bcaa00a6ed12fc62d05dc5e44521a1648fd2d07f#diff-1b06d4a1ccca0f0dff66d961923143a1R202)
  * Problem: use inconditionally `memmove` inside `__rcutils_copy_string()` inside `rcutils_set_error_state`. It implies dynamic memory allocation.
  * Dependecies:
    * rcl/rcl/include/rcl/error_handling.h
    * rcutils/src/security_directory.c
    * rcutils/src/time_unix.c
    * rcutils/src/split.c
    * rcutils/src/error_handling.c
    * rcutils/src/time.c
    * rcutils/src/string_map.c
    * rcutils/src/uint8_array.c
    * rcutils/src/hash_map.c
    * rcutils/src/char_array.c
    * rcutils/src/logging.c
    * rcutils/src/string_array.c
    * rcutils/src/array_list.c
    * rcutils/include/rcutils/types/array_list.h
    * rcutils/include/rcutils/types/hash_map.h
    * rcutils/include/rcutils/allocator.h
    * rcutils/include/rcutils/error_handling.h
    * rcl/rcl/include/rcl/error_handling.h
    * rmw/rmw/include/rmw/error_handling.h

* Macro **RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING** in `rcutils` error_handling.h:. [Check here](https://github.com/micro-ROS/rcutils/commit/bcaa00a6ed12fc62d05dc5e44521a1648fd2d07f#diff-1b06d4a1ccca0f0dff66d961923143a1R218)
  * Problem: use `RCUTILS_SET_ERROR_MSG`, see above.
  * Dependecies:
    * rcl/rcl/include/rcl/error_handling.h
    * rcutils/src/security_directory.c
    * rcutils/src/string_map.c
    * rcutils/src/logging.c
    * rcutils/include/rcutils/error_handling.h
    * rcl/rcl/include/rcl/error_handling.h
    * rmw/rmw/include/rmw/error_handling.h

### security_directory.c (moved from `rcl` to `rcutils`)

* Function **get_best_matching_directory** in `rcutils` security_directory.c [Check here](https://github.com/micro-ROS/rcutils/commit/9804287c3489ce9c88b714832abf54f9a7b7198d#diff-1ca0173d6a68ba1bdcd9ff908b769911R91)
  * Problem: use **tinydir** library which relies on filesytem
  * Dependecies:
    * rcutils/src/security_directory.c
    * rcl/rcl/src/rcl/security_directory.c

* Function **prefix_match_lookup**. [Check here](https://github.com/micro-ROS/rcutils/commit/9804287c3489ce9c88b714832abf54f9a7b7198d#diff-1ca0173d6a68ba1bdcd9ff908b769911L151)
  * Problem: use **tinydir** library which relies on filesytem
  * Dependencies:
    * rcutils/src/security_directory.c
    * rcl/rcl/src/rcl/security_directory.c

### allocator.c

* Function **rcutils_get_default_allocator**. [Check here](https://github.com/micro-ROS/rcutils/commit/3abb1eb2c9b206054101293997c0d4e541b1c657)
* Problem: use static allocated **default_allocator** which uses malloc/free. **rcutils_set_default_allocator** is proposed.
* Dependecies:
  * rcl/rcl/include/rcl/allocator.h
  * rcl/rcl/include/rcl/expand_topic_name.h
  * rcutils/src/allocator.c
  * rcutils/src/logging.c
  * rcutils/include/rcutils/types/string_map.h
  * rcutils/include/rcutils/types/array_list.h
  * rcutils/include/rcutils/types/string_array.h
  * rcutils/include/rcutils/types/hash_map.h
  * rcutils/include/rcutils/allocator.h
  * rmw_implementation/rmw_implementation/src/functions.cpp
  * rcl/rcl/include/rcl/allocator.h
  * rcl/rcl/include/rcl/expand_topic_name.h
  * rmw/rmw/src/allocators.c

### filesystem.c

* Functions **rcutils_get_cwd**, **rcutils_is_directory**, **rcutils_is_file**, **rcutils_exists**, **rcutils_is_readable**, **rcutils_is_writable** and **rcutils_is_readable_and_writable**. [Check here](https://github.com/micro-ROS/rcutils/commit/9804287c3489ce9c88b714832abf54f9a7b7198d)
  * Problem: rely on filesystem
  * Dependecies **rcutils_get_cwd()**:
    * rcutils/src/filesystem.c
    * rcutils/include/rcutils/filesystem.h
  * Dependecies **rcutils_is_directory()**:
    * rcutils/src/security_directory.c
    * rcutils/src/filesystem.c
    * rcutils/include/rcutils/filesystem.h
    * rcl/rcl/src/rcl/security_directory.c
  * Dependecies **rcutils_is_file()**:
    * rcutils/src/filesystem.c
    * rcutils/include/rcutils/filesystem.h
  * Dependencies **rcutils_exists()**:
    * rcutils/src/filesystem.c
    * rcutils/include/rcutils/filesystem.h
  * Dependencies **rcutils_is_readable()**:
    * rcutils/src/filesystem.c
    * rcutils/include/rcutils/filesystem.h
  * Dependencies **rcutils_is_writable()**:
    * rcutils/src/filesystem.c
    * rcutils/include/rcutils/filesystem.h
  * Dependencies **rcutils_is_readable_and_writable()**:
    * rcutils/src/filesystem.c
    * rcutils/include/rcutils/filesystem.h