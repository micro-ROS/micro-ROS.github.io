---
title: Adding Micro-ROS to a NuttX board configuration
permalink: /docs/tutorials/old/microros_nuttx_bsp/
---

<img src="https://img.shields.io/badge/Disclaimer-This_tutorial_is_unmaintained-red" style="display:inline"/>

**If you want to use Micro-ROS on a board that is not yet supported, this tutorial is for you!** However, we can only explain what you have to do on a board which is supported by NuttX already, that is, a board that has a board configuration. Writing a completely new board support package and configuration is beyond the scope of what the Micro-ROS project can teach.

### Caveats

1. The instructions in this tutorial have been tested on Linux only and since they use Linux shell commands, they will probably not work on Windows.
1. We have only ever used ARM-based boards. Boards using different microcontrollers might needs a different approach.
1. Our approach to adding C++ atomic swap instructions is problematic and needs further work, but it'll get you started.

### Basics

Compiling Micro-ROS for NuttX requires that the Board Configuration has a few C++ settings enabled. This tutorial explains what has to be added to an existing NuttX board configuration.

### Background: NuttX Board Configurations

**Note** This section is just for background, you don't need to create a board configuration yourself!

Microcontrollers are very diverse, and very versatile. The type and number of peripherals included differs a lot, and moreover, each specific board offers uses just a subset of them.

Therefore, the RTOS needs to be told which peripherals are used on a given board, and which pins the board has connected them to. In some cases, the board also requires custom initialization.

This is what we call the "board *configuration*". It differs from the so-called "Board Support *Package* (BSP)" which would contain the drivers for the micro-controller and its peripherals.

### Directory Structure

In NuttX up to version 7.x (which Micro-ROS currently uses), the configurations are stored in the `configs/` subdirectory.

In there are subdirectories for each board. The naming scheme differs, but often starts with the vendor, and then the name of the board. For example, the Olimex STM32-E407 reference board has its configuration in `configs/olimex-stm32e407`. We call this the *board base directory*.

Within the board base directory, there are two things:
 1) Board configuration directories, specifically `include`, `scripts`, and `src`.
 2) Predefined NuttX *build configurations*. These directories only have a `defconfig` file in them, and they are the directories you can pass to `scripts/configure.sh`.

## Adding Micro-ROS support

To add Micro-ROS build support, have two to 2 things:

 1) Enable the right C++ settings
 1) Add C++ atomics builtins

A good example of the necessary modifications can be found in commit [26917196](https://github.com/micro-ROS/NuttX/commit/26917196e744b22433e699af71da1fcb86a96623).

 ### Enabling the right C++ settings

All the compiler configuration is found in `scripts/Make.defs`. 

**Important**: Since this is a .defs file, it will only be evaluated by NuttX *during configuration*. Therefore, if you make changes, you have to do a `make distclean` and then a `tools/configure.sh` invocation!

1) Add C++ standard library includes

This basically means you have to add
`ARCHXXINCLUDES+=-isystem $(TOPDIR)/include/uClibc++`
after the block in which that variable is first defined.

2) Enable exceptions and RTTI

By default, the board configurations disable exceptions.

What we do is that we check whether UCLIBCXX_EXCEPTION is set, and if yes, we enable exceptions and RTTI. See [line 80 to 84 of commit 26917196](https://github.com/micro-ROS/NuttX/commit/26917196e744b22433e699af71da1fcb86a96623#diff-0199bac3041e59fbc59a9abd1492151eR80) for an example.
```makefile
ifeq ($(CONFIG_UCLIBCXX_EXCEPTION),y)
  ARCHCXXFLAGS = -fno-builtin -fcheck-new 
else
  ARCHCXXFLAGS = -fno-builtin -fno-exceptions -fcheck-new -fno-rtti
endif
```

3) Add libsupc++ to the build. This library is part of the toolchain, but there are multiple versions and you need the right one. The following snippet of Make code does that:
```makefile
LIBSUPXX = ${shell $(CC) $(CXXFLAGS) --print-file-name=libsupc++.a}
EXTRA_LIBPATHS = -L "${shell dirname "$(LIBSUPXX)"}"
EXTRA_LIBS = -lsupc++
```

### Add C++ atomics builtins

C++11 and up requires that the toolchain provides atomic swap operations. These are hardware specific and in the toolchain version we currently use, they are not yet available for ARM. Therefore, we add a compatibility file called `libatomic.c`

1. Copy `libatomic.c` from `configs/olimex-stm32e407/src/libatomic.c` to your board configuration `src` directory.-

1. Add `libatomic.c` to the `CSRCS` line of `src/Makefile`.