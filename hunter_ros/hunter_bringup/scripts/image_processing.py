import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import tqdm

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

def process_image_data(self, msg_data):
    front_cam_images = []
    for i in tqdm(range(len(msg_data['image_msg']))):
        img = np.fromstring(msg_data['image_msg'][i].data, np.uint8)
        img = cv2.imdecode(img, cv2.IMREAD_COLOR)
        # reshape image to 128x128
        img = cv2.resize(img, (128, 128), interpolation=cv2.INTER_AREA)
        front_cam_images.append(img)
    return front_cam_images

def process_bev_image_and_patches(self, msg_data):
    processed_data = {'image':[]}
    msg_data['patches'] = {}
    msg_data['patches_found'] = {}

    for i in tqdm(range(len(msg_data['image_msg']))):
        
            # written by anuj
        bevimage, _ = transform_compressed_image(msg_data['image_msg'][i])
        # bevimage, _ = DataProcessor.camera_imu_homography(msg_data['imu_msg'][i], msg_data['image_msg'][i])
        processed_data['image'].append(bevimage)

        # now find the patches for this image
        curr_odom = msg_data['odom_msg'][i]

        found_patch = False
        for j in range(i, max(i-30, 0), -2):
            prev_image = processed_data['image'][j]
            prev_odom = msg_data['odom_msg'][j]
            # cv2.imshow('src_image', processed_data['src_image'][i])
            patch, patch_black_pct, curr_img, vis_img = get_patch_from_odom_delta(
                curr_odom.pose.pose, prev_odom.pose.pose, curr_odom.twist.twist,
                prev_odom.twist.twist, prev_image, processed_data['image'][i])
            if patch is not None:
                found_patch = True
                if i not in msg_data['patches']:
                    msg_data['patches'][i] = []
                msg_data['patches'][i].append(patch)

            # stop adding more than 10 patches for a single data point
            if found_patch and len(msg_data['patches'][i]) > 5: break

        if not found_patch:
            print("Unable to find patch for idx: ", i)
            msg_data['patches'][i] = [processed_data['image'][i][500:564, 613:677]]

        # remove the i-30th image from RAM
        if i > 30:
            processed_data['image'][i-30] = None

        # was the patch found or no ?
        if found_patch: msg_data['patches_found'][i] = True
        else: msg_data['patches_found'][i] = False

    return msg_data['patches'], msg_data['patches_found']

def draw_arrow(arrow_start, arrow_end, color, inv_pos_transform, CENTER, vis_img):
    arrow_start_prev_frame = inv_pos_transform @ arrow_start
    arrow_end_prev_frame = inv_pos_transform @ arrow_end
    scaled_arrow_start = (arrow_start_prev_frame * 200).astype(np.int64)
    scaled_arrow_end = (arrow_end_prev_frame * 200).astype(np.int64)
    arrow_start_image_frame = CENTER + np.array((-scaled_arrow_start[1], -scaled_arrow_start[0]))
    arrow_end_image_frame = CENTER + np.array((-scaled_arrow_end[1], -scaled_arrow_end[0]))

    cv2.arrowedLine(
        vis_img,
        (arrow_start_image_frame[0], arrow_start_image_frame[1]),
        (arrow_end_image_frame[0], arrow_end_image_frame[1]),
        color,
        3
    )

def get_patch_from_odom_delta(curr_pos, prev_pos, curr_vel, prev_vel, prev_image, curr_image):
    curr_pos_np = np.array([curr_pos.position.x, curr_pos.position.y, 1])
    prev_pos_transform = np.zeros((3, 3))
    z_angle = R.from_quat([prev_pos.orientation.x, prev_pos.orientation.y, prev_pos.orientation.z, prev_pos.orientation.w]).as_euler('xyz', degrees=False)[2]
    prev_pos_transform[:2, :2] = R.from_euler('xyz', [0, 0, z_angle]).as_matrix()[:2,:2] # figure this out
    prev_pos_transform[:, 2] = np.array([prev_pos.position.x, prev_pos.position.y, 1]).reshape((3))

    inv_pos_transform = np.linalg.inv(prev_pos_transform)
    curr_z_angle = R.from_quat([curr_pos.orientation.x, curr_pos.orientation.y, curr_pos.orientation.z, curr_pos.orientation.w]).as_euler('xyz', degrees=False)[2]
    curr_z_rotation = R.from_euler('xyz', [0, 0, curr_z_angle]).as_matrix()
    projected_loc_np  = curr_pos_np + ACTUATION_LATENCY * (curr_z_rotation @ np.array([curr_vel.linear.x, curr_vel.linear.y, 0]))

    patch_corners = [
        projected_loc_np + curr_z_rotation @ np.array([0.3, 0.3, 0]),
        projected_loc_np + curr_z_rotation @ np.array([0.3, -0.3, 0]),
        projected_loc_np + curr_z_rotation @ np.array([-0.3, -0.3, 0]),
        projected_loc_np + curr_z_rotation @ np.array([-0.3, 0.3, 0])
    ]
    patch_corners_prev_frame = [
        inv_pos_transform @ patch_corners[0],
        inv_pos_transform @ patch_corners[1],
        inv_pos_transform @ patch_corners[2],
        inv_pos_transform @ patch_corners[3],
    ]
    scaled_patch_corners = [
        (patch_corners_prev_frame[0] * 206).astype(np.int64),
        (patch_corners_prev_frame[1] * 206).astype(np.int64),
        (patch_corners_prev_frame[2] * 206).astype(np.int64),
        (patch_corners_prev_frame[3] * 206).astype(np.int64),
    ]
    
    CENTER = np.array((640, 720))
    patch_corners_image_frame = [
        CENTER + np.array((-scaled_patch_corners[0][1], -scaled_patch_corners[0][0])),
        CENTER + np.array((-scaled_patch_corners[1][1], -scaled_patch_corners[1][0])),
        CENTER + np.array((-scaled_patch_corners[2][1], -scaled_patch_corners[2][0])),
        CENTER + np.array((-scaled_patch_corners[3][1], -scaled_patch_corners[3][0]))
    ]
    vis_img = prev_image.copy()

    # draw the patch rectangle
    cv2.line(
        vis_img,
        (patch_corners_image_frame[0][0], patch_corners_image_frame[0][1]),
        (patch_corners_image_frame[1][0], patch_corners_image_frame[1][1]),
        (0, 255, 0),
        2
    )
    cv2.line(
        vis_img,
        (patch_corners_image_frame[1][0], patch_corners_image_frame[1][1]),
        (patch_corners_image_frame[2][0], patch_corners_image_frame[2][1]),
        (0, 255, 0),
        2
    )
    cv2.line(
        vis_img,
        (patch_corners_image_frame[2][0], patch_corners_image_frame[2][1]),
        (patch_corners_image_frame[3][0], patch_corners_image_frame[3][1]),
        (0, 255, 0),
        2
    )
    cv2.line(
        vis_img,
        (patch_corners_image_frame[3][0], patch_corners_image_frame[3][1]),
        (patch_corners_image_frame[0][0], patch_corners_image_frame[0][1]),
        (0, 255, 0),
        2
    )
    # draw movement vector
    mov_start = curr_pos_np
    mov_end = curr_pos_np + (projected_loc_np - curr_pos_np) * 5

    head_start = curr_pos_np
    head_end = curr_pos_np + curr_z_rotation @ np.array([0.25, 0, 0])

    DataProcessor.draw_arrow(mov_start, mov_end, (255, 0, 0), inv_pos_transform, CENTER, vis_img)
    DataProcessor.draw_arrow(head_start, head_end, (0, 0, 255), inv_pos_transform, CENTER, vis_img)

    projected_loc_prev_frame = inv_pos_transform @ projected_loc_np
    scaled_projected_loc = (projected_loc_prev_frame * 200).astype(np.int64)
    projected_loc_image_frame = CENTER + np.array((-scaled_projected_loc[1], -scaled_projected_loc[0]))
    cv2.circle(vis_img, (projected_loc_image_frame[0], projected_loc_image_frame[1]), 3, (0, 255, 255))

    persp = cv2.getPerspectiveTransform(np.float32(patch_corners_image_frame), np.float32([[0, 0], [63, 0], [63, 63], [0, 63]]))

    patch = cv2.warpPerspective(
        prev_image,
        persp,
        (64, 64)
    )

    zero_count = np.logical_and(np.logical_and(patch[:, :, 0] == 0, patch[:, :, 1] == 0), patch[:, :, 2] == 0)

    if np.sum(zero_count) > PATCH_EPSILON:
        return None, 1.0, None, None

    return patch, (np.sum(zero_count) / (64. * 64.)), curr_image, vis_img
