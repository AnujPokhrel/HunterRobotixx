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
import time
from math import degrees
import os 

file_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..'))

#  This should be less hard coded...
# sys.path.append(file_path + "/include")
sys.path.append(file_path + "/include/micropython-fusion")
sys.path.append(file_path + "/include/libsbp/python/sbp/client")
# ROS Specfic
import rospy

from std_msgs.msg import Float32MultiArray

# Custom
from swift_pgm.msg import SwiftNavOutput

# IMU Fusion Libraries
from fusion import Fusion


from sbp.imu import SBP_MSG_IMU_RAW

class PGM():
    def __init__(self):
        rospy.loginfo_once("Swift IMU Abs ROS Node Started...")

        self.T = 0.01                # 10 Hz
        self.now = rospy.Time.now()
        self.header_frame_id = 'base_link'
        # Flags
        self.IMU_fusion     = False
        # Maintains the 10Hz refresh rate
        self.timer = rospy.Timer(rospy.Duration(self.T), self.timer_callback)
        
        # Subscriber
        self.socket_dump_sub    = rospy.Subscriber('swiftpgm/socket_dump', SwiftNavOutput, self.imuABSPublisher)

        # Publishers
        self.absolute_PRY_pub   = rospy.Publisher("swiftpgm/imu/abs", Float32MultiArray, queue_size=1)


    
    def startup(self, dump):
        raw_imu_data = dump.IMU_raw.data
        
        imu_data = {'acc_x': raw_imu_data[2],
                    'acc_y': raw_imu_data[3],
                    'acc_z': raw_imu_data[4],
                    'gyr_x': degrees(raw_imu_data[5]),
                    'gyr_y': degrees(raw_imu_data[6]),
                    'gyr_z': degrees(raw_imu_data[7])}

        # Start up Fusion
        self.timedef_func = lambda start, end: (end - start)
        self.fuse = Fusion()
        accel, gyro = self.IMUConversion(imu_data)
        start = time.perf_counter()  # Measure computation time only
        
        self.fuse.update_nomag(accel, gyro, self.timedef_func) # 979μs on Pyboard
        self.ts = self.timedef_func(time.perf_counter(), start)
        # print("Update time (uS):", t)
        self.IMU_fusion = True

    def imuABSPublisher(self, dump):
        # Uses the fusion library to provide Pitch, Roll, Yaw
        # Starting...
        if self.IMU_fusion:
            raw_imu_data = dump.IMU_raw.data
            
            imu_data = {'acc_x': raw_imu_data[2],
                        'acc_y': raw_imu_data[3],
                        'acc_z': raw_imu_data[4],
                        'gyr_x': degrees(raw_imu_data[5]),
                        'gyr_y': degrees(raw_imu_data[6]),
                        'gyr_z': degrees(raw_imu_data[7])}
            msg = Float32MultiArray()
            accel, gyro = self.IMUConversion(imu_data)
            
            self.fuse.update_nomag(accel, gyro)
            msg.data.append(self.fuse.pitch)
            msg.data.append(self.fuse.roll)
            msg.data.append(self.fuse.heading)

            self.absolute_PRY_pub.publish(msg)
        else:
            self.startup(dump)
    
    def IMUConversion(self, raw):
        # Input is the raw_IMU_data and conf message
        # Output is two tuples
        # Equations were provided by Swift Nav
        msg = raw

        acc_range   = 2
        gyro_range  = 2
        sf_acc  = 2. ** (acc_range + 1) / 2. ** 15
        sf_gyro = 2. ** (gyro_range + 1) / 2. ** 15

        accel_tmp   = (abs(msg['acc_x']), abs(msg['acc_y']), abs(msg['acc_z']))
        gyro_tmp    = (abs(msg['gyr_x']), abs(msg['gyr_y']), abs(msg['gyr_z']))

        accel   = [sf_acc * i for i in accel_tmp]
        gyro    = [sf_gyro * i for i in gyro_tmp]

        return accel, gyro

    def shutdown_trigger(self):
        rospy.loginfo_once("System told to shutdown...")
    
    def timer_callback(self, event):
        self.now = rospy.Time.now()

        

if __name__ == "__main__":
    # Init ROS Node
    rospy.init_node("imu_abs_node")

    pgm_obj = PGM()

    rospy.spin()

    rospy.on_shutdown(pgm_obj.shutdown_trigger)