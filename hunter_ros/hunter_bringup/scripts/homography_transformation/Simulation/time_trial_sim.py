import pickle
import cv2
import numpy as np
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import CompressedImage, Imu, Image
from geometry_msgs.msg import Twist
import rospy 
import message_filters 
import time
import math as m
#from hunter_msgs.msg import HunterStatus
from cv_bridge import CvBridge, CvBridgeError
import argparse
import threading

# a function to calculate the starting pixel of the line in y axis according to the velocity
# this is needed when the the bev image is generated dynamically
def calculate_pixel(velocity):
    '''calculate the value of y pixel from the bottom of the image according to the velocity
    size of the image is 1728x972
    Args:
        velocity: the velocity of the hunter
    Returns:
        the y pixel value'''
    #the equation is pixels = -31.2857 * velocity^2 + 271.7143 * velocity - 184.6
    pixel = 972 - (-31.2857 * velocity**2 + 271.7143 * velocity - 184.6)
    return pixel

class messageClass:
    def __init__(self):
        # print("initiazling message class")
        self.image_data = None
        self.applied_liner_velocity_x = 0.0
        self.applied_anguler_velocity_z = 0.0
        self.current_linear_velocity = 0.0
        self.current_angular_velocity = 0.0
        self.time_created = 0

class DataProcesor:
    def __init__(self, file_name):
        # print("initializing data processor")
        # self.odom = message_filters.Subscriber('/husky/husky_velocity_controller/odom', Odometry)
        self.vel_count = 0
        self.image_count = 0
        self.cmd_vel = Twist()

        cmd_vel = rospy.Subscriber('/husky/husky_velocity_controller/cmd_vel', Twist, self.cmd_vel_callback) 

        self.velocity = message_filters.Subscriber('/unity_command/ground_truth/husky/odom', Odometry)
        self.image = message_filters.Subscriber('/husky/camera/image', Image)
        
        #or we can use cache for the velocity, image and cmd_vel
        # self.odom = message_filters.Cache(self.odom, 20)
        # self.image = message_filters.Cache(self.odom, 20)
        # self.cmd_vel = message_filters.Cache(self.cmd_vel, 20)
        # self.velocity = message_filters.Cache(self.velocity, 20)
        
        self.ats = message_filters.ApproximateTimeSynchronizer([self.image, self.velocity], 20, 1.0, allow_headerless=True)
        self.ats.registerCallback(self.sensor_callback)

        self.timer = rospy.Timer(rospy.Duration(0.3), self.timer_callback)

        self.recent_data = None
        self.bridge = CvBridge()
        self.file = open(file_name, 'ab')
        self.data_dump = {}
        self.data_dump_lock = threading.Lock()

    
    def cmd_vel_callback(self, data):
        self.cmd_vel = data

    def sensor_callback(self, image, velocity):
        print("in the sensor callback")
        self.recent_data = (self.cmd_vel, image, velocity)

    def transform_image(self, image):
        # trying to transform compressed image
        # img = np.fromstring(image.data, np.uint8)
        # img = cv2.imdecode(img, cv2.IMREAD_COLOR)
        
        #transforming raw image
        img = self.bridge.imgmsg_to_cv2(image, "bgr8")
        #image height and width set
        IMAGE_H = 190
        IMAGE_W = 1000
        
        #resize the image to 0.9* 1080p
        img = cv2.resize(img, (1728, 972)) 
        trans_img = img.copy()

        #this is to draw the green line on top of the image.
        _0_0point = (300,790)
        _1_0point = (1300, 790)
        _0_1point = (575, 600)
        _1_1point = (1025, 600) 

        color = (0,255,0)
        thickness = 2
        cv2.line(img, _0_0point, _0_1point, color, thickness)
        cv2.line(img, _0_0point, _1_0point, color, thickness)
        cv2.line(img, _1_0point, _1_1point, color, thickness)
        cv2.line(img, _0_1point, _1_1point, color, thickness)
        cv2.imshow('original image', img)
        
        trans_img = trans_img[600:(600+IMAGE_H), 300:(300+IMAGE_W)] # Apply np slicing for ROI crop
        cv2.imshow('trans_img', trans_img)
        src = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [270, 0], [735, 0]])
        dst = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [0, 0], [IMAGE_W, 0]])
        further_shrink = np.float32([[175, IMAGE_H], [IMAGE_W-175, IMAGE_H], [0, 0], [IMAGE_W, 0]])
        M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
        Minv = cv2.getPerspectiveTransform(dst, further_shrink) # Inverse transformation


        warped_img = cv2.warpPerspective(trans_img, M, (IMAGE_W, IMAGE_H))
        img_inv = cv2.warpPerspective(warped_img, Minv, (IMAGE_W, IMAGE_H))
        # cv2.imshow('warped image', warped_img)
        cv2.imshow('inverse image', img_inv)
        cv2.waitKey(1)
        return img_inv 

    def timer_callback(self, event):
        if self.recent_data is not None:
            odom_msg, image_msg, velocity_msg = self.recent_data 
            #read image from the data
            bev_image = self.transform_image(image_msg)
            with self.data_dump_lock:
                print("saving new data")
                self.data_dump[str(time.time())] = {
                
                    'bev_image': bev_image,
                    'cmd_vel': self.recent_data[0],
                    'velocity_msg': self.recent_data[2],
                    # 'cmd_vel': self.recent_data[0]
            
                }


    def shutdown(self):
        with self.data_dump_lock:
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