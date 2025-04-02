#!/usr/bin/env python3
# Copyright (C) 2021 InDro Robotics
# Contact: austin.greisman@indrorobotics.com
#
# This source is subject to the license found in the file 'LICENSE' which must
# be be distributed together with this source. All other rights reserved.
#
"""
Created as a node to fix the issue of no GPS data messing up the EKF fusion.
"""
# General
from copy import copy
# ROS Specfic
import rospy
from nav_msgs.msg import Odometry

class OdomFix():
    def __init__(self):
        rospy.loginfo_once("Odom MUX ROS Node Started...")

        self.T = 0.0001               # 100 Hz
        self.now = rospy.Time.now()
        # Flags
        self.nav_sat_start      = False
        self.bunker_start       = False
        self.nav_sat_msg        = None
        self.bunker_msg         = None
        self.nav_last_time      = self.now
        self.bunker_last_time   = self.now

        # Maintains the 10Hz refresh rate
        self.timer = rospy.Timer(rospy.Duration(self.T), self.timer_callback)

        self.nav_sat_sub = rospy.Subscriber("/odometry/gps", Odometry, self.nav_sat_callback)
        self.bunker_sub = rospy.Subscriber("/bunker_odom", Odometry, self.bunker_callback)
        # Publishers
        self.socket_pub = rospy.Publisher('/odometry/gps_MUX', Odometry, queue_size=1)

    def nav_sat_callback(self, msg):
        self.nav_sat_msg = copy(msg)
        self.nav_last_time = self.now
        if not self.nav_sat_msg is None:
            self.nav_sat_start = True
        
    
    def bunker_callback(self, msg):
        self.bunker_msg = copy(msg)
        self.bunker_last_time   = self.now
        if not self.bunker_msg is None:
            self.bunker_start = True

    def shutdown_trigger(self):
        rospy.loginfo_once("System told to shutdown...")
    
    def timer_callback(self, event):
        self.now = rospy.Time.now()
        
        if self.nav_sat_start and (self.now - self.nav_last_time < rospy.Duration(1.0)):
            self.socket_pub.publish(self.nav_sat_msg)
            rospy.loginfo_throttle(2.5, "Publishing Nav Sat odom")
        elif self.bunker_start and (self.now - self.bunker_last_time < rospy.Duration(1.0)):
            self.socket_pub.publish(self.bunker_msg)
            rospy.logwarn_throttle(2.5, "NavSat message lost, using Bunker Odometry")
        else:
            rospy.logfatal_throttle(2.5, "No ODOM data for bunker or GPS...")
            self.shutdown_trigger()


if __name__ == "__main__":
    # Init ROS Node
    rospy.init_node("gps_mux_node")

    gps_obj = OdomFix()

    rospy.spin()

    rospy.on_shutdown(gps_obj.shutdown_trigger)
