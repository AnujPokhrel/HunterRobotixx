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
from math import sqrt
import sys
import time
import numpy as np

#  This should be less hard coded...
sys.path.append("/home/austin/Documents/SwiftPGM/libsbp/python/sbp/client")
sys.path.append("/home/austin/Documents/ws/src/swift_pgm/src/micropython-fusion")
# ROS Specfic
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu, Temperature
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32, Float32MultiArray

# IMU Fusion Libraries
from fusion import Fusion

# Swift specific
from drivers.pyserial_driver import PySerialDriver
from framer import Framer     # Converts Binary to SBP messages
from handler import Handler   # Handles SBP messages

from sbp.imu import SBP_MSG_IMU_RAW, SBP_MSG_IMU_AUX
from sbp.navigation import SBP_MSG_POS_LLH
from sbp.orientation import SBP_MSG_ORIENT_QUAT, SBP_MSG_BASELINE_HEADING
from sbp.vehicle import SBP_MSG_WHEELTICK

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
    
        self.IP = rospy.get_param("swift_IP", default="192.168.8.100")
        self.port = rospy.get_param("swift_port", default="55556")

        # Publishers
        self.gps_pub            = rospy.Publisher('gps', NavSatFix, queue_size=1)
        self.heading_pub        = rospy.Publisher('heading', Float32, queue_size=1)
        self.imu_pub            = rospy.Publisher('IMU/raw', Imu, queue_size=1)
        self.absolute_PRY_pub   = rospy.Publisher("IMU/abs", Float32MultiArray, queue_size=1)
        self.temperature_pub    = rospy.Publisher('Temp', Temperature, queue_size=1)


        # Starting...
        self.startup()
    
    def startup(self):
        # May have something more sophisticated later on...
        url = f'socket://{self.IP}:{self.port}'
        self.driver = PySerialDriver(url, baud=1000000).__enter__()
        self.source = Handler(Framer(self.driver.read, None, verbose=True)).__enter__()
        self.connected = True
        self.started = True

        # Start up Fusion
        self.timedef_func = lambda start, end: (end - start) / 1000000000
        self.fuse = Fusion()
        raw_imu_data = next(self.source.filter(SBP_MSG_IMU_RAW))[0]
        accel = (raw_imu_data.acc_x, raw_imu_data.acc_y, raw_imu_data.acc_z)
        gyro = (raw_imu_data.gyr_x, raw_imu_data.gyr_y, raw_imu_data.gyr_z)
        start = time.clock_gettime_ns(time.CLOCK_BOOTTIME)  # Measure computation time only
        
        self.fuse.update_nomag(accel, gyro, self.timedef_func) # 979μs on Pyboard
        self.ts = self.timedef_func(time.clock_gettime_ns(time.CLOCK_BOOTTIME), start)
        # print("Update time (uS):", t)
        self.IMU_fusion = True

    def imuPublisher(self):
        # Covarience can be added later...
        raw_imu_data = next(self.source.filter(SBP_MSG_IMU_RAW))[0]
        raw_imu_conf = next(self.source.filter(SBP_MSG_IMU_AUX))[0]
        # No filtering is going to happen here...
        msg = Imu()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.now

        accel_tmp, gyro_tmp = self.IMUConversion(raw_imu_data, raw_imu_conf)

        accel = Vector3()
        accel.x, accel.y, accel.z = accel_tmp

        gyro = Vector3()
        gyro.x, gyro.y, gyro.z = gyro_tmp

        msg.linear_acceleration = accel
        msg.angular_velocity = gyro

        self.imu_pub.publish(msg)

    def imuABSPublisher(self):
        # Uses the fusion library to provide Pitch, Roll, Yaw
        if self.IMU_fusion:
            raw_imu_data = next(self.source.filter(SBP_MSG_IMU_RAW))[0]
            raw_imu_conf = next(self.source.filter(SBP_MSG_IMU_AUX))[0]

            msg = Float32MultiArray()
            
            accel, gyro = self.IMUConversion(raw_imu_data, raw_imu_conf)
            
            self.fuse.update_nomag(accel, gyro, self.ts)
            msg.data.append(self.fuse.pitch)
            msg.data.append(self.fuse.roll)
            msg.data.append(self.fuse.heading)

            self.absolute_PRY_pub.publish(msg)

    def gpsPublisher(self):
        # Covariance can be added later...
        raw_gps_data = next(self.source.filter(SBP_MSG_POS_LLH))[0]
        # No filtering right now...

        if raw_gps_data.flags == 0 or raw_gps_data.n_sats == 0:
            rospy.loginfo_throttle_identical(2.5, "Waiting for GPS Lock")
        else:
            msg = NavSatFix()

            msg.header.frame_id = self.header_frame_id
            msg.header.stamp = self.now

            msg.latitude = raw_gps_data.lat
            msg.longitude = raw_gps_data.lon
            msg.altitude = raw_gps_data.height

            self.gps_pub.publish(msg)
    
    def headingPublisher(self):
        # System can only start once fusion has begun in Swift Nav
        if self.fusion_ready:
            raw_heading = next(self.source.filter(SBP_MSG_BASELINE_HEADING))[0]

            msg = Float32()
            msg.data = raw_heading.heading # In degrees - Relative to True North
        
            self.heading_pub.publish(msg)

    def temperaturePublisher(self):
        # Publishes temperature given from IMU
        raw_imu_conf = next(self.source.filter(SBP_MSG_IMU_AUX))[0]

        msg = Temperature()
        msg.header.frame_id = "base_link"
        msg.header.stamp = self.now

        msg.temperature = 25.0 + raw_imu_conf.temp / 256.0

        self.temperature_pub.publish(msg)
    
    def IMUConversion(self, raw, conf):
        # Input is the raw_IMU_data and conf message
        # Output is two tuples
        # Equations were provided by Swift Nav
        msg = raw

        acc_range   = conf.imu_conf & 0xF
        gyro_range  = conf.imu_conf >> 5
        sf_acc  = 2. ** (acc_range + 1) / 2. ** 15
        sf_gyro = 2. ** (gyro_range + 1) / 2. ** 15

        accel_tmp   = (abs(msg.acc_x), abs(msg.acc_y), abs(msg.acc_z))
        gyro_tmp    = (abs(msg.gyr_x), abs(msg.gyr_y), abs(msg.gyr_z))

        accel   = [sf_acc * i for i in accel_tmp]
        gyro    = [sf_gyro * i for i in gyro_tmp]

        return accel, gyro

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
            self.imuPublisher()
            # self.imuABSPublisher()
            self.gpsPublisher()
            self.headingPublisher()
            self.temperaturePublisher()
        
            raw_ticks = next(self.source.filter(SBP_MSG_WHEELTICK))[0]
            if raw_ticks.ticks == 0:
                rospy.loginfo_throttle(1.5, "Waiting for system to begin moving for heading...")
                self.fusion_ready = False
            else:
                self.fusion_ready = True

if __name__ == "__main__":
    # Init ROS Node
    rospy.init_node("swift_pgm_node")

    pgm_obj = PGM()

    rospy.spin()

    rospy.on_shutdown(pgm_obj.shutdown_trigger)