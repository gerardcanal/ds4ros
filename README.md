# ds4ros
Wrapper for ds4drv to use with ROS.

## Installation
Install [ds4drv](https://github.com/chrippa/ds4drv): sudo -H pip install git+https://github.com/gerardcanal/ds4drv (My fork is based on a fork that has some fixes for the led colors)

Set up the permissions as specified (here)[https://github.com/chrippa/ds4drv#permissions].

## Usage
You can use it either as a ROS node and supply a config file, or just use it as ds4drv with rosrun.
Example: `rosrun ds4ros ds4drv_wrapper.py --hidraw --led 00ff00`

The node only supports one controller at a time.

## ROS Services
- `ds4_ros/connect`
std_srvs/Empty
Connects to a controller

- `ds4_ros/disconnect`
std_srvs/Empty
Disconnects a controller (keeps waiting for reconnection)

- `ds4_ros/is_connected`
ds4ros/IsConnected
Checks if a controller is connected

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

