#!/usr/bin/env python3
import roslib
#roslib.load_manifest('amrl_msgs')
import os.path
import copy
from logging import root
import pickle
import numpy as np
from nav_msgs.msg import Odometry
from termcolor import cprint
import yaml
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R
import subprocess
#from amrl_msgs.msg import VescStateStamped
from sensor_msgs.msg import CompressedImage, Imu, Image
from hunter_msgs.msg import HunterStatus
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped, TwistStamped
from ublox_msgs.msg import NavPVT
import rospy 
import message_filters 
import time
import math as m
import argparse
import threading
import image_processing
import cv2

class DataProcessor:
    def __init__(self, file_name):

        self.velocity = message_filters.Subscriber('/calculated_velocity', TwistWithCovarianceStamped)
        # self.velocity = message_filters.Subscriber('/f9p_rover/fix_velocity', TwistWithCovarianceStamped)
        self.velocity.registerCallback(self.sensor_callback)
        self.image = message_filters.Subscriber('/rgb_publisher/color/image/compressed', CompressedImage)
        self.cmd_vel = message_filters.Subscriber('/stamped_cmd_vel', TwistStamped) 
        self.imu = message_filters.Subscriber('/swiftpgm/imu/raw', Imu)
        self.odom = message_filters.Subscriber('/odom', Odometry)

        #or we can use cache for the velocity, image and cmd_vel
        #self.velocity_cached = message_filters.Cache(self.velocity, 30, allow_headerless=True)
        self.image_cached = message_filters.Cache(self.image, 30, allow_headerless=True)
        self.cmd_vel_cached = message_filters.Cache(self.cmd_vel, 60, allow_headerless=True)
        self.imu_cached = message_filters.Cache(self.imu, 10, allow_headerless=True)
        self.odom_cached = message_filters.Cache(self.odom, 10, allow_headerless=True)

        self.recent_data = None
        self.file = open(file_name, 'ab')
        self.data_dump = {}
        self.data_dump_lock = threading.Lock()

    def sensor_callback(self, velocity):
        #get the latest image, cmd_vel and imu data with the closest timestamp as the velocity message and keep all of them in recent_data tuple
        self.recent_data = (velocity, self.image_cached.getElemBeforeTime(velocity.header.stamp), self.cmd_vel_cached.getElemBeforeTime(velocity.header.stamp), self.imu_cached.getElemBeforeTime(velocity.header.stamp))
        self.timer_callback() 
  
    def timer_callback(self):
        if self.recent_data is not None:
            image_msg = self.recent_data[1] 
            
            #read image from the data
            bev_image = image_processing.transform_compressed_image(image_msg)
            with self.data_dump_lock:
                print("####### saving new data #######")
                self.data_dump[str(time.time())] = {
                    'bev_image': bev_image,
                    'cmd_vel': self.recent_data[2],
                    'velocity_msg': self.recent_data[0],
                    'imu': self.recent_data[3]
                }
            
            self.recent_data = None
    
    #being used in the save data function, will probably need it
    @staticmethod
    def process_odom_vel_data(data):
        odoms = []
        for i in range(len(data['odom_msg'])-1):
            odom_now = data['odom_msg'][i]
            odom_now = np.array([odom_now.twist.twist.linear.x, odom_now.twist.twist.linear.y, odom_now.twist.twist.angular.z])
            if i>len(data['odom_msg'])-6:
                odom_next = data['odom_msg'][i+1]
            else:
                odom_next = data['odom_msg'][i+5] # assuming a delay of 0.2s
            odom_next = np.array([odom_next.twist.twist.linear.x, odom_next.twist.twist.linear.y, odom_next.twist.twist.angular.z])
            odoms.append(np.hstack((odom_now, odom_next)))
        return odoms

 