import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tqdm
from scipy.spatial.transform import Rotation as R
import time

def transform_uncompressed_image(image):
    bridge = CvBridge()
    
    #transforming raw image
    img = bridge.imgmsg_to_cv2(image, "bgr8")

    img_inv = actual_transformation(img)
    return img_inv, img

def transform_compressed_image(image):
    bridge = CvBridge()
    # trying to transform compressed image
    img = np.fromstring(image.data, np.uint8)
    img = cv2.imdecode(img, cv2.IMREAD_COLOR)

    img_inv = actual_transformation(img) 
    return img_inv, img

def actual_transformation(img):
    
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
    # cv2.imshow('original image', img)
    
    trans_img = trans_img[600:(600+IMAGE_H), 300:(300+IMAGE_W)] # Apply np slicing for ROI crop
    # cv2.imshow('trans_img', trans_img)
    src = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [270, 0], [735, 0]])
    dst = np.float32([[0, IMAGE_H], [IMAGE_W, IMAGE_H], [0, 0], [IMAGE_W, 0]])
    further_shrink = np.float32([[175, IMAGE_H], [IMAGE_W-175, IMAGE_H], [0, 0], [IMAGE_W, 0]])
    M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
    Minv = cv2.getPerspectiveTransform(dst, further_shrink) # Inverse transformation


    warped_img = cv2.warpPerspective(trans_img, M, (IMAGE_W, IMAGE_H))
    img_inv = cv2.warpPerspective(warped_img, Minv, (IMAGE_W, IMAGE_H))
    # cv2.imshow('warped image', warped_img)
    # cv2.imshow('inv Image', img_inv)
    cv2.waitKey(1)
    return img_inv 

#function to get bev image of the patch that is infont of the robot based on the odometry data
def transformation_from_odom(current_pos, previous_pos, current_vel, previous_vel, previous_image, current_image):
    IMAGE_H = 190
    IMAGE_W = 1000

    prev_M = None

    bird_eye_images = []
    
    img = previous_image 
    print(f"sizes of the image is {img.shape}")
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

    trans_img = trans_img[600:(600+IMAGE_H), 300:(300+IMAGE_W)] # Apply np slicing for ROI crop

    src = np.float32([[270, 0], [735, 0], [0, IMAGE_H], [IMAGE_W, IMAGE_H]])
    dst = np.float32([[0, 0], [IMAGE_W, 0], [0, IMAGE_H], [IMAGE_W, IMAGE_H]])

    #calculate incremental transformation based on odometry data
    z_angle_prev = R.from_quat([previous_pos.orientation.x, previous_pos.orientation.y, previous_pos.orientation.z, previous_pos.orientation.w]).as_euler('xyz', degrees=False)[2]
    z_angle_curr = R.from_quat([current_pos.orientation.x, current_pos.orientation.y, current_pos.orientation.z, current_pos.orientation.w]).as_euler('xyz', degrees=False)[2]
    delta_angle = z_angle_curr - z_angle_prev

    #calculate incremental translation based on odometry data
    delta_translation = np.array([current_pos.position.x - previous_pos.position.x, current_pos.position.y - previous_pos.position.y, 0])

    #calculate incremental scaling based on odometry data
    delta_scaling = np.array([current_vel.linear.x - previous_vel.linear.x, current_vel.linear.y - previous_vel.linear.y, 0])

    #calculate incremental transformation matrix
    delta_transform = np.zeros((3, 3))
    delta_transform[:2, :2] = R.from_euler('xyz', [0, 0, delta_angle]).as_matrix()[:2,:2]
    delta_transform[:, 2] = delta_translation + delta_scaling

    #from co-pilot
    #calculate incremental transformation matrix
    # if prev_M is None:
    #     prev_M = delta_transform
    # else:
    #     prev_M = prev_M @ delta_transform
    
    # #calculate inverse transformation matrix
    # inv_M = np.linalg.inv(prev_M)


    #from chatgpt
    M = cv2.getPerspectiveTransform(src, dst) # The transformation matrix
    M = np.matmul(M, delta_transform)
    
    warped_img = cv2.warpPerspective(trans_img, M, (IMAGE_W, IMAGE_H))
    cv2.imshow('current image', current_image)
    cv2.imshow('original image', img)
    cv2.imshow('warped image', warped_img)
    cv2.waitKey(1)
    time.sleep(1) 
    return warped_img 