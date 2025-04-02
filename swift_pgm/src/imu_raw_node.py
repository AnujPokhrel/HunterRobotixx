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
from math import degrees
import os 

file_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..'))

#  This should be less hard coded...
# sys.path.append(file_path + "/include")
sys.path.append(file_path + "/include/micropython-fusion")
sys.path.append(file_path + "/include/libsbp/python/sbp/client")
# ROS Specfic
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
#import quarternion for orientation
from geometry_msgs.msg import Quaternion

# Custom
from swift_pgm.msg import SwiftNavOutput

from sbp.imu import SBP_MSG_IMU_RAW, SBP_MSG_IMU_AUX
from sbp.navigation import SBP_MSG_POS_LLH
from sbp.orientation import SBP_MSG_ORIENT_QUAT, SBP_MSG_BASELINE_HEADING
from sbp.vehicle import SBP_MSG_WHEELTICK

class PGM():
    def __init__(self):
        rospy.loginfo_once("Swift IMU aw ROS Node Started...")

        self.T = 0.01                # 100 Hz
        self.now = rospy.Time.now()
        self.header_frame_id = 'swiftnav_link'
        # Flags
        self.connected      = False
        self.started        = False
        self.fusion_ready   = False
        self.IMU_fusion     = False
        self.call_count = 0
        self.offset_gyr_x = 0
        self.offset_gyr_y = 0
        self.offset_gyr_z = 0

        # Subscriber
        self.socket_dump_sub    = rospy.Subscriber('swiftpgm/socket_dump', SwiftNavOutput, self.imuPublisher)

        # Publishers
        self.imu_pub            = rospy.Publisher('swiftpgm/imu/raw', Imu, queue_size=1)
        
        # Maintains the 10Hz refresh rate
        self.timer = rospy.Timer(rospy.Duration(self.T), self.timer_callback)


    def imuPublisher(self, dump):
        if self.call_count < 10:
            print("Call count, dump: ",self.call_count, degrees(dump.IMU_raw.data[5]), degrees(dump.IMU_raw.data[6]), degrees(dump.IMU_raw.data[7]))
            self.call_count += 1
            self.offset_gyr_x += degrees(dump.IMU_raw.data[5])
            self.offset_gyr_y += degrees(dump.IMU_raw.data[6])
            self.offset_gyr_z += degrees(dump.IMU_raw.data[7])
        
        if self.call_count == 10:
            self.offset_gyr_x /= 10
            self.offset_gyr_y /= 10
            self.offset_gyr_z /= 10
            self.call_count += 1 
            print("offsets: ", self.offset_gyr_x, self.offset_gyr_y, self.offset_gyr_z)
            
        #print("in the imu publisher")
        # print(dump)
        # Covarience can be added later...
        raw_imu_data = dump.IMU_raw.data
 #       print(dump.IMU_raw.data)
        imu_data = {'acc_x': raw_imu_data[2],
                    'acc_y': raw_imu_data[3],
                    'acc_z': raw_imu_data[4],
                    'gyr_x': degrees(raw_imu_data[5]) - self.offset_gyr_x,
                    'gyr_y': degrees(raw_imu_data[6]) - self.offset_gyr_y,
                    'gyr_z': degrees(raw_imu_data[7]) - self.offset_gyr_z}
        msg = Imu()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.now

        accel_tmp, gyro_tmp, orientation_tmp = self.IMUConversion(imu_data)

        accel = Vector3()
        accel.x, accel.y, accel.z = accel_tmp

        gyro = Vector3()
        gyro.x, gyro.y, gyro.z = gyro_tmp

        orientation = Quaternion()
        orientation.x, orientation.y, orientation.z, orientation.w = orientation_tmp

        msg.linear_acceleration = accel
        msg.angular_velocity = gyro
        msg.orientation = orientation

        self.imu_pub.publish(msg)

    
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
        orientation_tmp = (0.33, 0.51, 8.32, 45)
        accel   = [sf_acc * i for i in accel_tmp]
        gyro    = [sf_gyro * i for i in gyro_tmp]

        return accel, gyro, orientation_tmp

    def shutdown_trigger(self):
        rospy.loginfo_once("System told to shutdown...")
    
    def timer_callback(self, event):
        self.now = rospy.Time.now()

if __name__ == "__main__":
    # Init ROS Node
    rospy.init_node("imu_raw_node")

    pgm_obj = PGM()

    rospy.spin()

    rospy.on_shutdown(pgm_obj.shutdown_trigger)