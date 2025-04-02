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
file_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..'))

#  This should be less hard coded...
# sys.path.append(file_path + "/include")
# sys.path.append(file_path + "/include/micropython-fusion")
# sys.path.append(file_path + "/include/libsbp/python/sbp/client")

# ROS Specfic
import rospy
from sensor_msgs.msg import NavSatFix

# Custom
from swift_pgm.msg import SwiftNavOutput

class PGM():
    def __init__(self):
        rospy.loginfo_once("Swift PGM ROS Node Started...")

        self.T = 0.01                # 10 Hz
        self.now = rospy.Time.now()
        self.header_frame_id = 'map'
        # Flags
        self.connected      = False
        self.started        = False
        self.fusion_ready   = False
        self.IMU_fusion     = False
        
        
        # Maintains the 10Hz refresh rate
        self.timer = rospy.Timer(rospy.Duration(self.T), self.timer_callback)
        
        # Subscriber
        self.socket_dump_sub    = rospy.Subscriber('swiftpgm/socket_dump', SwiftNavOutput, self.gpsPublisher)

        # Publishers
        self.gps_pub            = rospy.Publisher('swiftpgm/gps', NavSatFix, queue_size=1)


    def gpsPublisher(self, dump):
        # Covariance can be added later...
        raw_gps_data = dump.gps_raw.data
        raw_gps = {'lat': raw_gps_data[1],
                   'lon': raw_gps_data[2],
                   'height': raw_gps_data[3],
                #    'cov_n_n': raw_gps_data[4],
                #    'cov_e_e': raw_gps_data[5],
                #    'cov_d_d': raw_gps_data[6],
                #    'n_sats': raw_gps_data[7],
                #    'flags': raw_gps_data[8]}
                   'h_accuracy': raw_gps_data[4],
                   'v_accuracy': raw_gps_data[5],
                   'n_sats': raw_gps_data[6],
                   'flags': raw_gps_data[7]}
        # No filtering right now...

        if raw_gps['flags'] == 0 or raw_gps['n_sats'] == 0:
            rospy.loginfo_throttle_identical(2.5, "Waiting for GPS Lock")
        else:
            msg = NavSatFix()

            msg.header.frame_id = self.header_frame_id
            msg.header.stamp = self.now

            msg.latitude = raw_gps['lat']
            msg.longitude = raw_gps['lon']
            msg.altitude = raw_gps['height']

            # msg.position_covariance[0] = raw_gps['cov_n_n']
            # msg.position_covariance[4] = raw_gps['cov_e_e']
            # msg.position_covariance[8] = raw_gps['cov_d_d']
            msg.position_covariance[0] = (raw_gps['h_accuracy'] / 1000) ** 2 # In mm
            msg.position_covariance[4] = (raw_gps['h_accuracy'] / 1000) ** 2 # In mm
            msg.position_covariance[8] = (raw_gps['h_accuracy'] / 1000) ** 2 # In mm
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

            self.gps_pub.publish(msg)
    

    def shutdown_trigger(self):
        rospy.loginfo_once("System told to shutdown...")

    
    def timer_callback(self, event):
        self.now = rospy.Time.now()

if __name__ == "__main__":
    # Init ROS Node
    rospy.init_node("swift_gps_node")

    pgm_obj = PGM()

    rospy.spin()

    rospy.on_shutdown(pgm_obj.shutdown_trigger)