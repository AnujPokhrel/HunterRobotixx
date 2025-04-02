#this script takes the data from rostopics and saves in in a pickle file
import pickle
import numpy as np
from nav_msgs.msg import Odometry 
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

class DataProcesor:
    def __init__(self, file_name):

        #self.velocity = message_filters.Subscriber('/calculated_velocity', TwistWithCovarianceStamped)
        self.velocity = message_filters.Subscriber('/f9p_rover/fix_velocity', TwistWithCovarianceStamped)
        self.velocity.registerCallback(self.sensor_callback)
        self.image = message_filters.Subscriber('/rgb_publisher/color/image/compressed', CompressedImage)
        self.cmd_vel = message_filters.Subscriber('/stamped_cmd_vel', TwistStamped) 
        self.imu = message_filters.Subscriber('/swiftpgm/imu/raw', Imu)

        #or we can use cache for the velocity, image and cmd_vel
        #self.velocity_cached = message_filters.Cache(self.velocity, 30, allow_headerless=True)
        self.image_cached = message_filters.Cache(self.image, 30, allow_headerless=True)
        self.cmd_vel_cached = message_filters.Cache(self.cmd_vel, 60, allow_headerless=True)
        self.imu_cached = message_filters.Cache(self.imu, 10, allow_headerless=True)
 
        #self.ats = message_filters.ApproximateTimeSynchronizer([self.cmd_vel, self.image, self.velocity, self.imu], 100, 0.2, allow_headerless=True)
        #self.ats = message_filters.
        #self.ats.registerCallback(self.sensor_callback)

        #self.timer = rospy.Timer(rospy.Duration(0.2), self.timer_callback)

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
    
    def shutdown(self):
        with self.data_dump_lock:
            print("*********************** \n \n \n")
            print("shutting down")
            print(f"length of data dump is {len(self.data_dump)}")
            print(f"keys of the data dump is {self.data_dump.keys()}")
            print(f"dumping data")
            pickle.dump(self.data_dump, self.file)
            self.file.close()

#first trying to do stuff periodically.
def main(file_name):
    rospy.init_node('data_processor_node', anonymous=True)
    data_processor = DataProcesor(file_name)
    rospy.on_shutdown(data_processor.shutdown)
    rospy.spin()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-f', '--file', type=str, default='data.pkl', help='file to save data to')
    args = parser.parse_args()
    main(args.file)