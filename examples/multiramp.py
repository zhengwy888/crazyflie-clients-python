#!/usr/bin/env python
# This example shows how to control many copter using one (or many) Crazyradio dongles
# It is a slight modification from the ramp.py example

import time, sys
from threading import Thread

#FIXME: Has to be launched from within the example folder
#sys.path.append("../lib")
sys.path.append("../src/cflib")
import cflib
from cflib.crazyflie import Crazyflie

import logging
logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)


class Main:
    def __init__(self, uri):
        self.crazyflie = Crazyflie()
        self.uri = uri
        cflib.crtp.init_drivers(enable_debug_driver=False)
 
        # You may need to update this value if your Crazyradio uses a different frequency.
        self.crazyflie.open_link(uri)
        # Set up the callback when connected
        self.crazyflie.connected.add_callback(self.connectSetupFinished)
        self.crazyflie.disconnected.add_callback(self._disconnected)
        self.crazyflie.connection_failed.add_callback(self._connection_failed)
        self.crazyflie.connection_lost.add_callback(self._connection_lost)
 
    def connectSetupFinished(self, linkURI):
        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self.pulse_command).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print("Connection to %s failed: %s" % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print("Connection to %s lost: %s" % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print("Disconnected from %s" % link_uri)
 
    def pulse_command(self):
        thrust_mult = 1
        thrust_step = 500
        thrust = 20000
        pitch = 0
        roll = 0
        yawrate = 0
        # Unlock startup thrust protection
	print("unlock crazyflie")
        self.crazyflie.commander.send_setpoint(0, 0, 0, 0)
        while thrust >= 20000:
            self.crazyflie.commander.send_setpoint(roll, pitch, yawrate, thrust)
            time.sleep(0.1)
            if thrust >= 25000:
                thrust_mult = -1
            thrust += thrust_step * thrust_mult
        self.crazyflie.commander.send_setpoint(0, 0, 0, 0)
        # Make sure that the last packet leaves before the link is closed
        # since the message queue is not flushed before closing
        time.sleep(0.1)
	print("command Ended")
        self.crazyflie.close_link()

# Starting connection to both two copter. If a link using the same dongle
# is created, the communication will be shared with existing link
# Current implementation (as of Crazyradio 0.52) will divide the available
# bandwidth by 3 if used in that way so go easy on the log messages ;-)
# Future Crazyradio firmware will make that a bit more efficient
Main("radio://0/20/2M")
time.sleep(0.5)
Main("radio://0/40/2M")
