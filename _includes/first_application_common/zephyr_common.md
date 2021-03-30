Once the command is executed, a folder named `firmware` must be present in your workspace.

This step is in charge, among other things, of downloading a set of micro-ROS apps for the specific platform you are
addressing.
In the case of Zephyr, these are located at `firmware/zephyr_apps/apps`.
Each app is represented by a folder containing the following files:

* `src/main.c`: This file contains the logic of the application.
* `app-colcon.meta`: This file contains the micro-ROS app specific colcon configuration. Detailed info on how to
  configure the RMW via this file can be found
  [here](https://micro-ros.github.io/docs/tutorials/advanced/microxrcedds_rmw_configuration/).
* `CMakeLists.txt`: This is the CMake file containing the script to compile the application.
* `<transport>.conf`: This is a Zephyr specific and transport-dependent app configuration file.
`<transport>` can be `serial`, `serial-usb` and `host-udp`. 

For the user to create its custom application, a folder `<my_app>` will need to be registered in this location,
containing the four files just described.
