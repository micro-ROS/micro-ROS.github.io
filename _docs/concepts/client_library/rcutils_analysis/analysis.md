# rcutils micro-ROS analysis

- [rcutils micro-ROS analysis](#rcutils-micro-ros-analysis)
  - [error_handling.h](#errorhandlingh)
  - [security_directory.c](#securitydirectoryc)
  - [allocator.c](#allocatorc)
  - [filesystem.c](#filesystemc)

*All dependencies include own header and implementation file.*

*Test folders are excluded.*

*security_filesystem appears both in rcl and rcutils due to the proposed location change.*

## error_handling.h

- Macro **RCUTILS_SAFE_FWRITE_TO_STDERR**. [Check here](https://github.com/micro-ROS/rcutils/commit/5bb92d91aae01feef046c05077997a686d35fe56#diff-1b06d4a1ccca0f0dff66d961923143a1L42)
  - Problem: uses **fwrite()** 
  - Dependecies:
    - /uros/rcl/rcl/src/rcl/expand_topic_name.c
    - /uros/rcl/rcl/src/rcl/logging_rosout.c
    - /uros/rcl/rcl/src/rcl/logging.c
    - /uros/rcl/rcl/src/rcl/context.c
    - /uros/rcl/rcl_yaml_param_parser/src/parser.c
    - /uros/rcutils/src/split.c
    - /uros/rcutils/src/error_handling.c
    - /uros/rcutils/src/allocator.c
    - /uros/rcutils/src/logging.c
    - /uros/rcutils/src/error_handling_helpers.h
    - /uros/rcutils/include/rcutils/logging.h
    - /uros/rcutils/include/rcutils/error_handling.h
    - /ros2/rcl/rcl/src/rcl/expand_topic_name.c
    - /ros2/rcl/rcl/src/rcl/logging_rosout.c
    - /ros2/rcl/rcl/src/rcl/logging.c
    - /ros2/rcl/rcl/src/rcl/context.c
    - /ros2/rcl/rcl_yaml_param_parser/src/parser.c
    - /ros2/rmw/rmw/include/rmw/error_handling.h

- Macro **RCUTILS_SET_ERROR_MSG**. [Check here](https://github.com/micro-ROS/rcutils/commit/5bb92d91aae01feef046c05077997a686d35fe56#diff-1b06d4a1ccca0f0dff66d961923143a1L200)
  - Problem: uses **__rcutils_copy_string()** 
  - Dependecies:
    - /uros/rcl/rcl/include/rcl/error_handling.h
    - /uros/rcutils/src/security_directory.c
    - /uros/rcutils/src/time_unix.c
    - /uros/rcutils/src/split.c
    - /uros/rcutils/src/error_handling.c
    - /uros/rcutils/src/time.c
    - /uros/rcutils/src/string_map.c
    - /uros/rcutils/src/uint8_array.c
    - /uros/rcutils/src/hash_map.c
    - /uros/rcutils/src/char_array.c
    - /uros/rcutils/src/logging.c
    - /uros/rcutils/src/string_array.c
    - /uros/rcutils/src/array_list.c
    - /uros/rcutils/include/rcutils/types/array_list.h
    - /uros/rcutils/include/rcutils/types/hash_map.h
    - /uros/rcutils/include/rcutils/allocator.h
    - /uros/rcutils/include/rcutils/error_handling.h
    - /ros2/rcl/rcl/include/rcl/error_handling.h
    - /ros2/rmw/rmw/include/rmw/error_handling.h

- Macro **RCUTILS_SET_ERROR_MSG_WITH_FORMAT_STRING**. [Check here](https://github.com/micro-ROS/rcutils/commit/5bb92d91aae01feef046c05077997a686d35fe56#diff-1b06d4a1ccca0f0dff66d961923143a1L212)
  - Problem: uses **rcutils_snprintf()** 
  - Dependecies:
    - /uros/rcl/rcl/include/rcl/error_handling.h
    - /uros/rcutils/src/security_directory.c
    - /uros/rcutils/src/string_map.c
    - /uros/rcutils/src/logging.c
    - /uros/rcutils/include/rcutils/error_handling.h
    - /ros2/rcl/rcl/include/rcl/error_handling.h
    - /ros2/rmw/rmw/include/rmw/error_handling.h

## security_directory.c

- Function **get_best_matching_directory()**. [Check here](https://github.com/micro-ROS/rcutils/commit/9b6c4d6b0e954d4a84a1ae41aa7657aa876ecf23#diff-1ca0173d6a68ba1bdcd9ff908b769911L87)
  - Problem: uses **tinydir** library
  - Dependecies:
    - /uros/rcutils/src/security_directory.c
    - /ros2/rcl/rcl/src/rcl/security_directory.c

- Function **prefix_match_lookup()**. [Check here](https://github.com/micro-ROS/rcutils/commit/9b6c4d6b0e954d4a84a1ae41aa7657aa876ecf23#diff-1ca0173d6a68ba1bdcd9ff908b769911L151)
  - Problem: uses **_TINYDIR_FILENAME_MAX** macro from **tinydir** library
  - Dependencies:
    - /uros/rcutils/src/security_directory.c
    - /ros2/rcl/rcl/src/rcl/security_directory.c

## allocator.c

- Function **rcutils_get_default_allocator()**. [Check here](https://github.com/micro-ROS/rcutils/commit/1a7fde263eae6ddffbabfd6ac22b62b93afc153f#diff-e2e6ccf38de409700323df9674d18304L72)
  - Problem: uses static allocated **default_allocator** which uses malloc/free
  - Dependecies:
    - /uros/rcl/rcl/include/rcl/allocator.h
    - /uros/rcl/rcl/include/rcl/expand_topic_name.h
    - /uros/rcutils/src/allocator.c
    - /uros/rcutils/src/logging.c
    - /uros/rcutils/include/rcutils/types/string_map.h
    - /uros/rcutils/include/rcutils/types/array_list.h
    - /uros/rcutils/include/rcutils/types/string_array.h
    - /uros/rcutils/include/rcutils/types/hash_map.h
    - /uros/rcutils/include/rcutils/allocator.h
    - /ros2/rmw_implementation/rmw_implementation/src/functions.cpp
    - /ros2/rcl/rcl/include/rcl/allocator.h
    - /ros2/rcl/rcl/include/rcl/expand_topic_name.h
    - /ros2/rmw/rmw/src/allocators.c

## filesystem.c

- Functions **rcutils_get_cwd()**, **rcutils_is_directory()**, **rcutils_is_file()**, **rcutils_exists()**, **rcutils_is_readable()**, **rcutils_is_writable()** and **rcutils_is_readable_and_writable()**. [Check here](https://github.com/micro-ROS/rcutils/commit/5a94c101ae6cec65f0c4caae0fa9e2e5a0b2d39a)
  - Problem: uses **#include <sys/stat.h>** which is filesystem related
  - Dependecies **rcutils_get_cwd()**:
    - /uros/rcutils/src/filesystem.c
    - /uros/rcutils/include/rcutils/filesystem.h
  - Dependecies **rcutils_is_directory()**:
    - /uros/rcutils/src/security_directory.c
    - /uros/rcutils/src/filesystem.c
    - /uros/rcutils/include/rcutils/filesystem.h
    - /ros2/rcl/rcl/src/rcl/security_directory.c
  - Dependecies **rcutils_is_file()**:
    - /uros/rcutils/src/filesystem.c
    - /uros/rcutils/include/rcutils/filesystem.h
  - Dependencies **rcutils_exists()**:
    - /uros/rcutils/src/filesystem.c
    - /uros/rcutils/include/rcutils/filesystem.h
  - Dependencies **rcutils_is_readable()**:
    - /uros/rcutils/src/filesystem.c
    - /uros/rcutils/include/rcutils/filesystem.h
  - Dependencies **rcutils_is_writable()**:
    - /uros/rcutils/src/filesystem.c
    - /uros/rcutils/include/rcutils/filesystem.h
  - Dependencies **rcutils_is_readable_and_writable()**:
    - /uros/rcutils/src/filesystem.c
    - /uros/rcutils/include/rcutils/filesystem.h

Dependencies searching command:

```bash
grep -Hnr "[Command here]" ./uros ./ros2 | sed -E "s/(.+)\:([0-9]+)\:(.+)/   - \1/" | uniq
```