# ds4ros
Wrapper for ds4drv to use with ROS.

## Installation
Install (ds4drv)[https://github.com/chrippa/ds4drv]: sudo -H pip3 install git+https://github.com/gerardcanal/ds4drv (My fork is based on a fork that has some fixes for the led colors)

Set up the permissions as specified (here)[https://github.com/chrippa/ds4drv#permissions].

## ROS Services
- `ds4_ros/connect`
std_srvs/Empty
Connects to a controller

- `ds4_ros/disconnect`
std_srvs/Empty
Disconnects a controller (keeps waiting for reconnection)

- `ds4_ros/set_color`
ds4ros/SetColor
Sets the lightbar to the specified RGB color

- `ds4_ros/battery_status`
ds4ros/BatteryStatus
Returns the status of the battery of the controller (charge and plugged).

- `ds4_ros/flash`
ds4ros/StartFlash
Sets the flashing of the LED light

- `ds4_ros/stop_flash`
std_srvs/Empty
Stops the flashing of the LED

- `ds4_ros/rumble`
ds4ros/Rumble
Activates the controller rumble motors

- `ds4_ros/stop_rumble`
std_srvs/Empty
Stops the rumble

