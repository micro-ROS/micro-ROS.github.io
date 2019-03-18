---
title: Debugging with Visual Studio Code
permalink: /docs/tutorials/advanced/debugging-vscode/
author: Ingo Lütkebohle
---

This is a follow-up to [Debugging with gdb and openocd](/docs/tutorials/debugging-gdb-openocd/), because the set up done in that tutorial is a pre-requisite to debugging with Visual Studio Code.

## Motivation

Visual Studio Code is a modern IDE that is very easy to extend and popular with both the Web/Cloud and IoT communities. It is also one of the easiest IDEs to get working with embedded systems. That said, it is *not* the most powerful or featureful IDEs for this purpose, but it is easy and will do.

## Prerequisites

 * All the prerequisites of [Debugging with gdb and openocd](/docs/tutorials/debugging-gdb-openocd/)
 * Cortex-M hardware (all of our standard boards are ARM based)
 * [Visual Studio Code](https://code.visualstudio.com/)


## Installing Cortex-Debug

In the extensions marketplace, enter "cortex", then install "Cortex-Debug". Depending on your version of Visual Studio Code, you may need to restart after installing the extension.

## Set up your project for debugging

Open your project folder in Visual Studio Code -- this is usually the `NuttX` folder, or a subdirectory of `apps`.

### Create a Visual Studio Code launch configuration for NuttX

From the `Debug` menu, select `Open Configurations`. This will open a `launch.json' file. See [Cortex-Debug Launch configurations](https://marcelball.ca/projects/cortex-debug/cortex-debug-launch-configurations/) for documentation.

To get started, I have prepared a working launch configuration for using our STM32-E407 board with the ARM-USB-OCD-H jtag probe. If you use a different board or probe, you only need to replace the `configFiles` section. Each entry in the section is an argument that you would normally pass as a `-f` option to OpenOCD.
```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug (OpenOCD/NuttX)",
            "cwd": "${workspaceRoot}",
            "executable": "nuttx",
            "request": "launch",
            "type": "cortex-debug",
            "servertype": "openocd",
            "device": "stm32f4x",
            "configFiles": [
                "interface/ftdi/olimex-arm-usb-ocd-h.cfg",
                "target/stm32f4x.cfg"
            ]
        }
    ]
}
```
The `name` is what will appear in the status bar for running it.

### Running the debugger

Either press `F5` or select `Debug/Start Debugging` from the menu to get started. This will take a moment, and then you should get a red status bar and the debug window, like in the following image:
![debug window](/img/tutorials/debug-vscode.png)

As in the gdb tutorial, initially you won't see much because the program is stopped during OS initialization. Press `F5` again or click the "play" button from the debug menu at the top of the window, wait a few seconds, then press `F6` or click the pause button. The window should change to give you thread, variable, and register information, like in the following. Note that "Call Stack" window displays multiple threads.

![debug window with running code](/img/tutorials/debug-vscode-phyread.png)

### Adding an SVD File

You may have noticed that on the left-hand side, there is a sub-window called "Cortex Peripherals" which simply states "No SVD File loaded". SVD means "System View Description" and is a standard format which microcontroller vendors use to describe the available features of their MCUs. 

For example, in the case of our STM32-E407 board, which features an STM32F407ZGT6 MCU, we can download the SVD description from [STM's web-page for the STM32F407ZG series](https://www.st.com/en/microcontrollers-microprocessors/stm32f407zg.html). In the "HW Model, CAD Libraries & SVD", you will find a link to the [STM32F4 series SVD](https://www.st.com/resource/en/svd/stm32f4_svd.zip). 

Extract the SVD and then add an `svdFile` attribute to the launch configuration. The full configuration will look like this:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Debug (OpenOCD/SVD)",
            "cwd": "${workspaceRoot}",
            "executable": "nuttx",
            "request": "launch",
            "type": "cortex-debug",
            "servertype": "openocd",
            "device": "stm32f4x",
            "svdFile": "STM32F407.svd",
            "configFiles": [
                "interface/ftdi/olimex-arm-usb-ocd-h.cfg",
                "target/stm32f4x.cfg"
            ]
        }
    ]
}
```

Run the debugger again, and your window should look as follows:
![](/img/tutorials/debug-vscode-svd.png)

Voilà! The `Cortex Peripherals` is populated with everything that the STM32F407 MCU has to offer. Please note that not all of these peripherals might actually be connected on the board. However, those that are, and that are used in your application, can easily be investigated like this.
