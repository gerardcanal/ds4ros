#!/usr/bin/env python3
# Gerard Canal, HAT-King's College London, 2019
# For python3 install: 
#       sudo apt-get install python3-pip python3-yaml
#       sudo pip3 install rospkg catkin_pkg
# Starts ds4drv with the config and sets the param for joy

import rospy
import glob
import signal
import sys
import ds4drv
from ds4drv.__main__ import SigintHandler, create_controller_thread
from ds4drv.actions import ActionRegistry
from ds4drv.config import load_options
from ds4drv.backends import BluetoothBackend, HidrawBackend
from ds4drv.daemon import Daemon
from threading import Thread
from std_srvs.srv import Empty, EmptyResponse
from ds4ros.srv import Rumble, SetColor, StartFlash, BatteryStatus, BatteryStatusResponse
from ds4ros.srv import IsConnected, IsConnectedResponse
from std_msgs.msg import Bool


class ROSDS4Controller:
    def __init__(self, cfg_path=None):  # Adapted from ds4drv __main__:main()
        self.threads = []

        self.sigint_handler = SigintHandler(self.threads)
        signal.signal(signal.SIGINT, self.sigint_handler)

        try:
            if cfg_path:
                sys.argv.extend(['--config', cfg_path])
            sys.argv = [a for a in sys.argv if (not a.startswith('_') and ":=" not in a) ]
            self.options = load_options()
        except ValueError as err:
            Daemon.exit("Failed to parse options: {0}", err)

        if self.options.hidraw:
            self.backend = HidrawBackend(Daemon.logger)
        else:
            self.backend = BluetoothBackend(Daemon.logger)

        try:
            self.backend.setup()
        except BackendError as err:
            Daemon.exit(err)

        if len(self.options.controllers) > 1:
            rospy.logwarn(
                '[ROSDS4Controller] Only one controller is processed. Multiple controller options will be ignored')

        self.thread = None
        self.disconnect_called = True
        self.start_ros_services()

    def connect(self):
        if not self.thread or not self.thread.is_alive():
            self.thread = create_controller_thread(1, self.options.controllers[0])
            if self.thread not in self.threads:
                self.threads.append(self.thread)
        self.device = next(self.backend.devices)  # Gets device
        self.thread.controller.setup_device(self.device)
        self.disconnect_called = False
        return self.device

    def disconnect(self):
        self.thread.controller.cleanup_device()
        self.disconnect_called = True

    def is_connected(self):
        return self.thread.controller.device is not None

    def get_jsdev(self):
        i = ActionRegistry.actions.index(ds4drv.actions.input.ReportActionInput)
        return self.thread.controller.actions[i].joystick.joystick_dev

    def get_battery(self, _):
        i = ActionRegistry.actions.index(ds4drv.actions.status.ReportActionStatus)
        lr = self.thread.controller.actions[i]._last_report
        max_value = lr.plug_usb and ds4drv.actions.status.BATTERY_MAX_CHARGING or ds4drv.actions.status.BATTERY_MAX
        battery = 100 * lr.battery / float(max_value)
        res = BatteryStatusResponse()
        res.charge_percent = battery
        res.charging = lr.plug_usb
        return res

    def set_color(self, r, g, b):
        self.device.set_led(r, g, b)

    def srv_set_color(self, req):
        self.set_color(int(req.color.r), int(req.color.g), int(req.color.b))
        return []

    def start_led_flash(self, req):
        self.stop_led_flash()
        self.device.start_led_flash(req.on_time, req.off_time)
        return ()

    def stop_led_flash(self, _=''):
        self.device.stop_led_flash()
        return EmptyResponse()

    def rumble(self, req):
        self.device.rumble(req.small, req.big)
        return ()

    def stop_rumble(self, _):
        self.device.rumble(0, 0)
        return ()

    def srv_connect(self, _):
        if not self.is_connected():
            self.connect()
        else:
            raise Exception('Controller is already connected!')
        return ()

    def srv_disconnect(self, _):
        if self.is_connected():
            self.disconnect()
        else:
            raise Exception('Controller is not connected!')
        return ()

    def srv_is_connected(self, _):
        res = IsConnectedResponse()
        res.is_connected = self.is_connected()
        return res

    def start_ros_services(self):
        self.connectsrv = rospy.Service('~connect', Empty, self.srv_connect)
        self.disconnectsrv = rospy.Service('~disconnect', Empty, self.srv_disconnect)
        self.color = rospy.Service('~set_color', SetColor, self.srv_set_color)
        self.flash = rospy.Service('~flash', StartFlash, self.start_led_flash)
        self.stopflash = rospy.Service('~stop_flash', Empty, self.stop_led_flash)
        self.rumble = rospy.Service('~rumble', Rumble, self.rumble)
        self.stoprumble = rospy.Service('~stop_rumble', Empty, self.stop_rumble)
        self.battery = rospy.Service('~battery_status', BatteryStatus, self.get_battery)
        self.isconnected = rospy.Service('~is_connected', IsConnected, self.srv_is_connected)


def main():
    cfg_path = rospy.get_param('~ds4drv_cfg', '')
    rospy.loginfo('[ROSDS4Controller] Starting ds4drv with config: ' + cfg_path)

    controller = ROSDS4Controller(cfg_path)

    if controller.options.hidraw:
        rospy.loginfo(
            '[ROSDS4Controller] Waiting for joystick to connect before attempting hidraw connection through ds4drv...')
        while not glob.glob('/dev/input/js*'):
            rospy.sleep(1)
            if rospy.is_shutdown():
                rospy.loginfo('[ROSDS4Controller] Shutting down node!')
                return None

    controller.connect()

    # Set dev parameter
    dev = controller.get_jsdev()
    rospy.loginfo('[ROSDS4Controller] Setting device parameter to: ' + dev)
    rospy.set_param('joy_node/dev', dev)

    # Wait forever
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        if not controller.is_connected() and not controller.disconnect_called:
            rospy.logwarn('[ROSDS4Controller] Controller connection lost! Trying to reconnect...')
            controller.connect()
        rate.sleep()

    rospy.delete_param('joy_node/dev')  # Delete the param
    controller.disconnect()


if __name__ == '__main__':
    rospy.init_node('ds4_ros', anonymous=False)
    main()
