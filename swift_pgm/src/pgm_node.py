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
from subprocess import run, PIPE
from datetime import datetime

#file_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '..'))
#  This should be less hard coded...
#sys.path.append(file_path + "/include")
#sys.path.append(file_path + "/include/micropython-fusion")
#sys.path.append(file_path + "/include/libsbp/python/sbp/client")
# ROS Specfic
import rospy
from std_msgs.msg import Float64MultiArray

# Custom Message
from swift_pgm.msg import SwiftNavOutput
from ping3 import ping
# IMU Fusion Libraries
#from fusion import Fusion

# Swift specific
from sbp.client.drivers.pyserial_driver import PySerialDriver
from sbp.client import Handler, Framer

from sbp.imu import SBP_MSG_IMU_RAW, SBP_MSG_IMU_AUX
from sbp.navigation import SBP_MSG_POS_LLH, SBP_MSG_POS_LLH_GNSS, SBP_MSG_POS_LLH_COV
from sbp.orientation import SBP_MSG_ORIENT_QUAT, SBP_MSG_BASELINE_HEADING
from sbp.vehicle import SBP_MSG_WHEELTICK

class PGM():
    def __init__(self):
        rospy.loginfo_once("Swift PGM ROS Node Started...")

        self.T = 0.001               # 100 Hz
        self.now = rospy.Time.now()
        # Flags
        self.connected      = False
        self.started        = False


        # Maintains the 10Hz refresh rate
        self.timer = rospy.Timer(rospy.Duration(self.T), self.timer_callback)
    
        self.IP = rospy.get_param("~swift_IP", default="192.168.42.152")
        self.port = rospy.get_param("~swift_port", default="55556")

        # Publishers
        self.socket_pub = rospy.Publisher('swiftpgm/socket_dump', SwiftNavOutput, queue_size=1)
        # Starting...
        self.startup()
    
    def startup(self):
        # Ensure SwiftNav is up
        response = 256
        while response == False:
            response = ping(self.IP)
            rospy.loginfo_throttle(2.5, "Waiting for SwiftNav to come online... May take a minute..")
            rospy.sleep(5)

        rospy.sleep(1)

        # SwitnNav is up, make sure Jetson time is correct before starting node... Systems to cause odd behaviour        
        continue_flag = True
        #while continue_flag:
        #    google_time = run('''wget -qSO- --max-redirect=0 google.com 2>&1 | grep Date: | cut -d' ' -f5-6''', shell=True, stdout=PIPE, stderr=PIPE).stdout.decode("utf-8").strip()
        #    system_time = datetime.now().strftime("%d %b")
        #    if system_time == google_time:
        #        continue_flag = False
        #    else:
        #        rospy.loginfo_throttle(2.5, "System time and true time is different... Waiting..")
        #        rospy.sleep(5)
        
        rospy.loginfo_throttle(2.5, "System time and true time is the same and Swift is up!")

        # May have something more sophisticated later on...
        url = f'socket://{self.IP}:{self.port}'
        self.driver = PySerialDriver(url, baud=1000000).__enter__()
        self.source = Handler(Framer(self.driver.read, None, verbose=True)).__enter__()
        self.connected = True
        self.started = True

    def socketPublisher(self):
        if self.connected and self.started:
            # print("Publishing SwiftNav Data")
            # print(next(self.source.filter(SBP_MSG_IMU_RAW))[0])
            # Covarience can be added later...
            raw_imu_data = next(self.source.filter(SBP_MSG_IMU_RAW))[0]
            #raw_imu_conf = next(self.source.filter(SBP_MSG_IMU_AUX))[0]
            #raw_gps_data = next(self.source.filter(SBP_MSG_POS_LLH_COV))[0]
            # raw_gps_data = next(self.source.filter(SBP_MSG_POS_LLH))[0]

            # No filtering is going to happen here...
            msg = SwiftNavOutput()
            msg.header.stamp = self.now
            IMU_conf = Float64MultiArray()
            IMU_raw = Float64MultiArray()
            gps_raw = Float64MultiArray()

            IMU_raw.data.append(raw_imu_data.tow)
            IMU_raw.data.append(raw_imu_data.tow_f)
            IMU_raw.data.append(raw_imu_data.acc_x)
            IMU_raw.data.append(raw_imu_data.acc_y)
            IMU_raw.data.append(raw_imu_data.acc_z)
            IMU_raw.data.append(raw_imu_data.gyr_x)
            IMU_raw.data.append(raw_imu_data.gyr_y)
            IMU_raw.data.append(raw_imu_data.gyr_z)
            
            
            #gps_raw.data.append(raw_gps_data.tow)
            # gps_raw.data.append(raw_gps_data.lat)
            # gps_raw.data.append(raw_gps_data.lon)
            # gps_raw.data.append(raw_gps_data.height)
            # gps_raw.data.append(raw_gps_data.cov_n_n)
            # gps_raw.data.append(raw_gps_data.cov_e_e)
            # gps_raw.data.append(raw_gps_data.cov_d_d)
            # gps_raw.data.append(raw_gps_data.h_accuracy)
            # gps_raw.data.append(raw_gps_data.v_accuracy)
            # gps_raw.data.append(raw_gps_data.n_sats)
            # gps_raw.data.append(raw_gps_data.flags)
            
            # print(f"printing the IMU_RAW array: {IMU_raw}")
            #msg.IMU_conf_raw    = IMU_conf
            msg.IMU_raw         = IMU_raw
            # msg.gps_raw         = gps_raw

            self.socket_pub.publish(msg)

    def shutdown_trigger(self):
        rospy.loginfo_once("System told to shutdown...")
        #Shutdown stuff
        if self.started:
            self.driver.flush()
            self.driver.close()
    
    def timer_callback(self, event):
        self.now = rospy.Time.now()

        if not self.connected:
            rospy.loginfo_throttle(2.5, "Waiting to connect to Swift PGM")
            if not self.started:
                rospy.loginfo_throttle(2.5, "Waiting for system to start... May take a few minutes...")
        
        if self.connected and self.started:
            self.socketPublisher()


if __name__ == "__main__":
    # Init ROS Node
    rospy.init_node("swift_info_dump_node")

    pgm_obj = PGM()

    rospy.spin()

    rospy.on_shutdown(pgm_obj.shutdown_trigger)
