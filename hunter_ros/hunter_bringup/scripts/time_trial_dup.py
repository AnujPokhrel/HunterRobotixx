import pickle
import cv2
import numpy as np
from nav_msgs.msg import Odometry 
from sensor_msgs.msg import CompressedImage, Imu, Image
import rospy 
import message_filters 
import time
import math as m
from hunter_msgs.msg import HunterStatus
from cv_bridge import CvBridge, CvBridgeError

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
    def __init__(self):
        # print("initializing data processor")
        self.odom = message_filters.Subscriber('/odom', Odometry)

        self.velocity = message_filters.Subscriber('/hunter_status', HunterStatus)
        # /zed2/zed_node/right/image_rect_color
        self.image = message_filters.Subscriber('/zed2/zed_node/right_raw/image_raw_color', Image)
        # print(self.image)
        # print(type(self.image))
        self.ats = message_filters.ApproximateTimeSynchronizer([self.odom, self.image, self.velocity], 20, 1.0, allow_headerless=True)
        self.ats.registerCallback(self.sensor_callback)

        self.timer = rospy.Timer(rospy.Duration(0.3), self.timer_callback)

        self.recent_data = None
        self.bridge = CvBridge()
        self.file = open('data.pkl', 'wb')

def sensor_callback(odom, image):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(image, "bgr8")
    # cv2.imshow('original image', img)
    transform_image(img)

def transform_image(img):
    # print("trying to transform image")
    #img = np.fromstring(image.data, np.uint8)
    #img = cv2.imdecode(img, cv2.IMREAD_COLOR)
    #image height and width set
    
    IMAGE_H = 190
    IMAGE_W = 1000
    # cv2.imshow('original image', img)
    #resize the image to 0.9* 1080p
    
    img = cv2.resize(img, (1728, 972)) 
    _0_0point = (300,790)
    _1_0point = (1300, 790)
    _0_1point = (575, 600)
    _1_1point = (1025, 600) 

    color = (0,255,0)
    thickness = 2
    trans_img = img.copy()
    #this is to draw the green line on top of the image.
    cv2.line(img, _0_0point, _0_1point, color, thickness)
    cv2.line(img, _0_0point, _1_0point, color, thickness)
    cv2.line(img, _1_0point, _1_1point, color, thickness)
    cv2.line(img, _0_1point, _1_1point, color, thickness)
    cv2.imshow('original image', img)
    trans_img = trans_img[600:(600+IMAGE_H), 300:(300+IMAGE_W)] # Apply np slicing for ROI crop
    # cv2.imshow('trans_img', trans_img)
    src = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [270, 0], [735, 0]])
    dst = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [0, 0], [IMAGE_W, 0]])
    further_shrink = np.float32([[175, IMAGE_H], [IMAGE_W-175, IMAGE_H], [0, 0], [IMAGE_W, 0]])
    M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
    Minv = cv2.getPerspectiveTransform(dst, further_shrink) # Inverse transformation


    warped_img = cv2.warpPerspective(trans_img, M, (IMAGE_W, IMAGE_H))
    img_inv = cv2.warpPerspective(warped_img, Minv, (IMAGE_W, IMAGE_H))
    cv2.waitKey(1)
    #show img_inv
    # cv2.imshow('transformed image', img_inv)
    return img_inv 

def timer_callback(recent_data):
    bridge = CvBridge()
    if recent_data is not None:
        odom_msg, image_msg, velocity_msg = recent_data 
        #read image from the data
        bev_image = transform_image(image_msg)
        cv_image = bridge.compressed_imgmsg_to_cv2(bev_image, "bgr8")

        # print("writing data")
        # print(type(cv_image))
        # pickle.dump((cv_image, odom_msg.twist.twist.linear.x, odom_msg.twist.twist.angular.z, velocity_msg.linear_velocity, velocity_msg.steering_angle, time.time()), self.file)

def shutdown(file):
    print("shutting down")
    file.close()

def callback(imu, image):
    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(image, "bgr8")
    img = cv2.resize(img, (1728, 972))   
    cv2.imshow('disp', img)
    cv2.waitKey(1)

#first trying to do stuff periodically.
def listener():
    rospy.init_node('data_processor_node', anonymous=True)
    # data_processor = DataProcesor()
   # print("initializing data processor")
    imu = message_filters.Subscriber('/zed2/zed_node/imu/data', Imu)
    # /zed2/zed_node/right/image_rect_color
    image = message_filters.Subscriber('/zed2/zed_node/right_raw/image_raw_color', Image)
    ts = message_filters.ApproximateTimeSynchronizer([imu, image], 20, 1.0, allow_headerless=True)
    ts.registerCallback(sensor_callback)
    # timer = rospy.Timer(rospy.Duration(0.3), timer_callback)

    recent_data = None
    file = open('data.pkl', 'wb')

    # rospy.on_shutdown(shutdown(file))
    rospy.spin()

if __name__ == '__main__':
    listener()