#!/usr/bin/env python3
# Copyright (C) 2021 InDro Robotics
# Contact: austin.greisman@indrorobotics.com
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
"""
Created as a node to publish odom, IMU, and GPS data from the Swift PGM unit
"""

# General Libraries
import sys
import os 
import time
file_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..'))

#  This should be less hard coded...
# sys.path.append(file_path + "/include")
sys.path.append(file_path + "/include/micropython-fusion")
sys.path.append(file_path + "/include/libsbp/python/sbp/client")
# ROS Specfic
import rospy

from std_msgs.msg import Float32


# Swift specific
from drivers.pyserial_driver import PySerialDriver
from framer import Framer     # Converts Binary to SBP messages
from handler import Handler   # Handles SBP messages

from sbp.orientation import SBP_MSG_ORIENT_QUAT, SBP_MSG_BASELINE_HEADING
from sbp.vehicle import SBP_MSG_WHEELTICK

class PGM():
    def __init__(self):
        rospy.loginfo_once("Swift PGM ROS Node Started...")

        self.T = 0.01                # 10 Hz
        self.now = rospy.Time.now()
        self.header_frame_id = 'maps'
        # Flags
        self.connected      = False
        self.started        = False
        self.fusion_ready   = False

        # Maintains the 10Hz refresh rate
        self.timer = rospy.Timer(rospy.Duration(self.T), self.timer_callback)
    
        self.IP = rospy.get_param("~swift_IP", default="192.168.8.100")
        self.port = "55555"

        # Publishers
        self.heading_pub        = rospy.Publisher('swiftpgm/heading', Float32, queue_size=1)

        # Starting...
        self.startup()
    
    def startup(self):
        response = 256
        while response != 0:
            response = os.system("ping -c 1 -w2 " + self.IP + " > /dev/null 2>&1")
            rospy.loginfo_throttle(2.5, "Waiting for SwiftNav to come online... May take a minute..")
            time.sleep(5)
        # May have something more sophisticated later on...
        url = f'socket://{self.IP}:{self.port}'
        self.driver = PySerialDriver(url, baud=1000000).__enter__()
        self.source = Handler(Framer(self.driver.read, None, verbose=True)).__enter__()
        self.connected = True
        self.started = True

    
    def headingPublisher(self):
        # System can only start once fusion has begun in Swift Nav
        if self.fusion_ready:
            raw_heading = next(self.source.filter(SBP_MSG_BASELINE_HEADING))[0]

            msg = Float32()
            msg.data = raw_heading.heading # In degrees - Relative to True North
        
            self.heading_pub.publish(msg)

    def shutdown_trigger(self):
        rospy.loginfo_once("System told to shutdown...")
        #Shutdown stuff
        if self.started:
            self.driver.flush()
            self.driver.close()
    
    def timer_callback(self, event):
        self.now = rospy.Time.now()

        if not self.connected:
            rospy.loginfo_throttle(1.5, "Waiting to connect to Swift PGM")
            if not self.started:
                rospy.loginfo_throttle(1.5, "Waiting for system to start... May take a few minutes...")
        
        if self.connected and self.started:
            if not self.fusion_ready:
                raw_ticks = next(self.source.filter(SBP_MSG_WHEELTICK))[0]
                if raw_ticks.ticks == 0:
                    rospy.loginfo_throttle(1.5, "Waiting for system to begin moving for heading...")
                    self.fusion_ready = False
                else:
                    self.fusion_ready = True

if __name__ == "__main__":
    # Init ROS Node
    rospy.init_node("swift_heading_node")

    pgm_obj = PGM()

    rospy.spin()

    rospy.on_shutdown(pgm_obj.shutdown_trigger)