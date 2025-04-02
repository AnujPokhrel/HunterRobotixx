#!/usr/bin/env python3.6
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage, Imu, Image
import rospy
import message_filters
import cv2
from scipy.spatial.transform import Rotation as R

def homography_camera_displacement(R1, R2, t1, t2, n1):
    R12 = R2 @ R1.T
    t12 = R2 @ (- R1.T @ t1) + t2
    # d is distance from plane to t1.
    d = np.linalg.norm(n1.dot(t1.T))

    H12 = R12 - ((t12 @ n1.T) / d)
    H12 /= H12[2, 2]
    return H12

def callback_haresh(imu, image):
    orientation_quat = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
   #hk
    # z_correction = odom.pose.pose.position.z

    C_i = np.array(
        [622.0649233612024, 0.0, 633.1717569157071, 0.0, 619.7990184421728, 368.0688607187958, 0.0, 0.0, 1.0]).reshape(
        (3, 3))

    R_imu_world = R.from_quat(orientation_quat)

    R_imu_world = R_imu_world.as_euler('xyz', degrees=True)
    R_imu_world[0], R_imu_world[1] = -R_imu_world[0], R_imu_world[1]
    R_imu_world[2] = 0.

    R_imu_world = R_imu_world
    R_imu_world = R.from_euler('xyz', R_imu_world, degrees=True)

    R_cam_imu = R.from_euler("xyz", [90, -90, 0], degrees=True)
    R1 = R_cam_imu * R_imu_world
    R1 = R1.as_matrix()

    R2 = R.from_euler("xyz", [0, 0, -90], degrees=True).as_matrix()
    t1 = R1 @ np.array([0., 0., 0.5]).reshape((3, 1))
    t2 = R2 @ np.array([-2.5, -0., 6.0]).reshape((3, 1))
    n = np.array([0, 0, 1]).reshape((3, 1))
    n1 = R1 @ n

    H12 = homography_camera_displacement(R1, R2, t1, t2, n1)
    homography_matrix = C_i @ H12 @ np.linalg.inv(C_i)
    homography_matrix /= homography_matrix[2, 2]

    img = np.fromstring(image.data, np.uint8)
    img = cv2.imdecode(img, cv2.IMREAD_COLOR)

    output = cv2.warpPerspective(img, homography_matrix, (1280, 720))

    img = cv2.resize(img, (640, 360))
    output = cv2.resize(output, (640, 360))
    cv2.imshow('disp', np.hstack((img, output)))
    cv2.waitKey(1)

def callback(imu, image):
    IMAGE_H = 80
    IMAGE_W = 460
    bridge = CvBridge()
    #img = np.fromstring(image.data, np.uint8)
    #img = cv2.imdecode(img, cv2.IMREAD_COLOR)
    img = bridge.imgmsg_to_cv2(image, "bgr8")
    org_img = img.copy()
    #img = img[442:(442+IMAGE_H), 450:(450+IMAGE_W)] # Apply np slicing for ROI crop

    #src = np.float32([[0, IMAGE_H], [460, IMAGE_H], [80, 0], [380, 0]])
#    dst = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [0, 0], [IMAGE_W, 0]])
#    further_shrink = np.float32([[80, IMAGE_H], [IMAGE_W-80, IMAGE_H], [0, 0], [IMAGE_W, 0]])
#    M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
#   Minv = cv2.getPerspectiveTransform(dst, further_shrink) # Inverse transformation

    
#    warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H))
    img = cv2.resize(img, (1728, 972))   
    cv2.imshow('disp', img)
    # cv2.imshow('orginal Image', org_img)
#    img_inv = cv2.warpPerspective(warped_img, Minv, (IMAGE_W, IMAGE_H))
    # cv2.imshow('inv Image', img_inv)
#    org_img = cv2.resize(org_img, (640, 360))
#    img_inv = cv2.resize(img_inv, (640, 360))
    #cv2.imshow('disp', np.hstack((org_img, img)))   
    cv2.waitKey(1)

# def callback(imu, image):
#     IMAGE_H = 80
#     IMAGE_W = 380

#     src = np.float32([[0, IMAGE_H], [380, IMAGE_H], [92, 0], [IMAGE_W, 0]])
#     dst = np.float32([[569, IMAGE_H], [711, IMAGE_H], [0, 0], [IMAGE_W, 0]])
#     M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
#     Minv = cv2.getPerspectiveTransform(dst, src) # Inverse transformation

#     img = np.fromstring(image.data, np.uint8)
#     img = cv2.imdecode(img, cv2.IMREAD_COLOR)
#     img = img[442:(442+IMAGE_H), 450:(450+IMAGE_W)] # Apply np slicing for ROI crop
#     warped_img = cv2.warpPerspective(img, M, (IMAGE_W, IMAGE_H)) # Image warping
    
#     img_inv = cv2.warpPerspective(warped_img, Minv, (IMAGE_W, IMAGE_H)) # Inverse transformation
#     cv2.imshow('img', img) # Show results
#     # cv2.imshow('warped_img', warped_img)
#     # cv2.imshow('img_inv', img_inv)
    #ap
#     cv2.waitKey(1)

def listener():
    rospy.init_node('test_homography_bag', anonymous=True)
    # imu = message_filters.Subscriber('/vectornav/IMU', Imu)
    #imu but not really imu, it's odom
    imu = message_filters.Subscriber('/zed2/zed_node/imu/data', Imu)
    # /zed2/zed_node/right/image_rect_color
    image = message_filters.Subscriber('/zed2/zed_node/right_raw/image_raw_color', Image)
    ts = message_filters.ApproximateTimeSynchronizer([imu, image], 20, 1.0, allow_headerless=True)
    # ts.registerCallback(callback_haresh)
    ts.registerCallback(callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

