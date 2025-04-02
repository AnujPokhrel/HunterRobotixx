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
        self.havenot_published  = True
        self.nav_sat_msg        = None
        self.bunker_msg         = None
        self.bunker_msg_old     = None
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

    def topic_publisher(self):
        # The goal is to take in the bunker odom, then subtract that value and add the nav sat value
        # This will allow the nav sat to be in the same frame as the odom
        self.published_msg = Odometry()
        if (not self.bunker_msg_old is None) and (not self.nav_sat_msg is None):
            self.published_msg = self.subtract_odom(self.bunker_msg, self.bunker_msg_old, self.nav_sat_msg)
        else:
            self.published_msg = self.bunker_msg
        self.socket_pub.publish(self.published_msg)
        rospy.loginfo_throttle(2.5, "NavSat message lost, publishing offset bunker odom")

    def subtract_odom(self, a, b, c):
        # Subtract each element of the three odom messages
        y = Odometry()
        y.header.stamp = a.header.stamp
        y.header.frame_id = a.header.frame_id
        y.pose.pose.position.x = a.pose.pose.position.x - b.pose.pose.position.x + c.pose.pose.position.x
        y.pose.pose.position.y = a.pose.pose.position.y - b.pose.pose.position.y + c.pose.pose.position.y
        y.pose.pose.position.z = a.pose.pose.position.z - b.pose.pose.position.z + c.pose.pose.position.z
        y.pose.pose.orientation.x = a.pose.pose.orientation.x - b.pose.pose.orientation.x + c.pose.pose.orientation.x
        y.pose.pose.orientation.y = a.pose.pose.orientation.y - b.pose.pose.orientation.y + c.pose.pose.orientation.y
        y.pose.pose.orientation.z = a.pose.pose.orientation.z - b.pose.pose.orientation.z + c.pose.pose.orientation.z
        y.pose.pose.orientation.w = a.pose.pose.orientation.w - b.pose.pose.orientation.w + c.pose.pose.orientation.w
        y.twist.twist.linear.x = a.twist.twist.linear.x - b.twist.twist.linear.x + c.twist.twist.linear.x
        y.twist.twist.linear.y = a.twist.twist.linear.y - b.twist.twist.linear.y + c.twist.twist.linear.y
        y.twist.twist.linear.z = a.twist.twist.linear.z - b.twist.twist.linear.z + c.twist.twist.linear.z
        y.twist.twist.angular.x = a.twist.twist.angular.x - b.twist.twist.angular.x + c.twist.twist.angular.x
        y.twist.twist.angular.y = a.twist.twist.angular.y - b.twist.twist.angular.y + c.twist.twist.angular.y
        y.twist.twist.angular.z = a.twist.twist.angular.z - b.twist.twist.angular.z + c.twist.twist.angular.z
        return y
    
    def shutdown_trigger(self):
        rospy.loginfo_once("System told to shutdown...")
    
    def timer_callback(self, event):
        self.now = rospy.Time.now()
        
        if self.nav_sat_start and (self.now - self.nav_last_time < rospy.Duration(1.0)):
            self.socket_pub.publish(self.nav_sat_msg)
            rospy.loginfo_throttle(2.5, "Publishing Nav Sat odom")
            self.havenot_published = True # Reset the flag
        elif self.bunker_start and (self.now - self.bunker_last_time < rospy.Duration(1.0)):
            # self.socket_pub.publish(self.bunker_msg)
            if self.havenot_published:
                self.bunker_msg_old = self.bunker_msg
                self.havenot_published = False
            self.topic_publisher()
        else:
            rospy.logfatal_throttle(2.5, "No ODOM data for bunker or GPS...")
            self.shutdown_trigger()


if __name__ == "__main__":
    # Init ROS Node
    rospy.init_node("gps_mux_node")

    gps_obj = OdomFix()

    rospy.spin()

    rospy.on_shutdown(gps_obj.shutdown_trigger)
