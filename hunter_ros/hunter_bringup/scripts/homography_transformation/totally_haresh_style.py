#got this from haresh karnan.
#!/usr/bin/env python3
import roslib
#roslib.load_manifest('amrl_msgs')
import os.path
import copy
from logging import root
import pickle
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import CompressedImage, Joy, Imu
from geometry_msgs.msg import TwistStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import message_filters
from termcolor import cprint
import yaml
from tqdm import tqdm
from scipy.spatial.transform import Rotation as R
import subprocess
#from amrl_msgs.msg import VescStateStamped
import image_processing
import argparse
import time

PATCH_SIZE = 64
PATCH_EPSILON = 0.5 * PATCH_SIZE * PATCH_SIZE

# TODO -very very important, need this figure this shiz out  
ACTUATION_LATENCY = 0.1

class ListenRecordData:
    def __init__(self, save_data_path, rosbag_play_process):
        self.data = [] 

        #self.config_path = config_path
        self.save_data_path = save_data_path
        self.rosbag_play_process = rosbag_play_process

        self.odom_msgs = np.zeros((200, 3), dtype=np.float32)
        
        self.batch_idx = 0
        self.counter = 0

        velocity = message_filters.Subscriber('/calculated_velocity', TwistWithCovarianceStamped)
        image = message_filters.Subscriber('/rgb_publisher/color/image/compressed', CompressedImage)
        cmd_vel = message_filters.Subscriber('/stamped_cmd_vel', TwistStamped) 
        imu = message_filters.Subscriber('/swiftpgm/imu/raw', Imu)
        odom = message_filters.Subscriber('/odom', Odometry)

        #or we can use cache for the velocity, image and cmd_vel
        self.odom_cached = message_filters.Cache(odom, 30, allow_headerless=True)
        self.image_cached = message_filters.Cache(image, 30, allow_headerless=True)
        self.cmd_vel_cached = message_filters.Cache(cmd_vel, 10, allow_headerless=True)
        self.imu_cached = message_filters.Cache(imu, 10, allow_headerless=True)
        
        velocity.registerCallback(self.callback)

        # subscribe to odom topic to record the last 1 second of odom data
        rospy.Subscriber('/odom', Odometry, self.odom_callback) # 200hz
        # rospy.Subscriber('/camera/gyro/sample', Imu, self.gyro_callback) # 60hz


        self.msg_data = {
            'image_msg': [],
            'src_image': [],
            'odom_msg': [],
            #'joystick_msg': [],
            'odom_1sec_msg': [],
            # 'accel_msg': [],
            # 'gyro_msg': [],
            'imu_msg': [],
            'velocity_msg': [],
            'cmd_vel_msg': []
            # 'vesc_drive_msg': [],
            # 'sensor_core_msg': []
        }

        self.open_thread_lists = []


    def callback(self, velocity):
        print('Received messages :: ', self.counter, ' ___')
        
        self.msg_data['image_msg'].append(self.image_cached.getElemBeforeTime(velocity.header.stamp))
        self.msg_data['odom_msg'].append(self.odom_cached.getElemBeforeTime(velocity.header.stamp))
        self.msg_data['imu_msg'].append(self.imu_cached.getElemBeforeTime(velocity.header.stamp))
        self.msg_data['odom_1sec_msg'].append(self.odom_msgs.flatten())
        self.msg_data['velocity_msg'].append(velocity)
        self.msg_data['cmd_vel_msg'].append(self.cmd_vel_cached.getElemBeforeTime(velocity.header.stamp))
        self.counter += 1

    def odom_callback(self, msg):
        # add to the queue self.odom_msgs
        self.odom_msgs = np.roll(self.odom_msgs, -1, axis=0)
        msg = msg.twist.twist
        self.odom_msgs[-1] = np.array([msg.linear.x, msg.linear.y, msg.angular.z])

    def save_data(self, msg_data, batch_idx):
        data = {}

        # process front camera image
        print('Processing front camera image')
        data['front_cam_image'] = self.process_image_data(msg_data)

        # process odom_1_sec data
        print('Processing odom_1_sec data')
        data['odom_1sec_msg'] = self.msg_data['odom_1sec_msg']
        # data['accel_msg'] = self.msg_data['accel_msg']
        # data['gyro_msg'] = self.msg_data['gyro_msg']
        del msg_data['odom_1sec_msg']
        # del msg_data['accel_msg']
        # del msg_data['gyro_msg']

        # process odom
        print('Processing odom data')
        data['odom'] = self.process_odom_vel_data(msg_data)
        data['imu_msg'] = msg_data['imu_msg']
        data['velocity_msg'] = msg_data['velocity_msg']
        data['cmd_vel_msg'] = msg_data['cmd_vel_msg']
        data['odom_1sec_msg'] = data['odom_1sec_msg'][:len(data['odom'])]
        # data['accel_msg'] = data['accel_msg'][:len(data['odom'])]
        # data['gyro_msg'] = data['gyro_msg'][:len(data['odom'])]
        data['front_cam_image'] = data['front_cam_image'][:len(data['odom'])]

        data['patches'], data['patches_found'] = self.process_bev_image_and_patches(msg_data)
        data['patches'].pop(len(data['patches'].keys())-1)
        data['patches_found'].pop(len(data['patches_found'].keys())-1)

        del msg_data['image_msg']
        del msg_data['odom_msg']
        del msg_data['imu_msg']

        assert(len(data['odom']) == len(data['patches'].keys()))
        patches = []
        sorted_keys = sorted(data['patches'].keys())
        for i in range(len(sorted_keys)):
            patches.append(data['patches'][sorted_keys[i]])
        data['patches'] = patches

        # dont save imu_msg
        # del data['imu_msg']

        # asserts
        data_length = len(data['odom'])
        cprint('data length: '+str(data_length), 'green', attrs=['bold'])
        # assert(len(data['joystick']) == data_length)
        assert(len(data['patches']) == data_length)
        assert(len(data['patches_found']) == data_length)
        assert(len(data['odom_1sec_msg']) == data_length)
        # assert(len(data['velocity_msg']) == data_length)
        # assert(len(data['cmd_vel_msg']) == data_length)
        # assert(len(data['imu_msg']) == data_length)
    
        if len(data['odom']) > 0:
            # save data
            cprint('Saving data...{}'.format(len(data['odom'])), 'yellow')
            path = os.path.join(self.save_data_path, 'data_{}.pkl'.format(batch_idx))
            pickle.dump(data, open(path, 'wb'))
            cprint('Saved data successfully ', 'yellow', attrs=['blink'])
    
    # extracts images from numpy data
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
        found_patch_number = 0

        for i in tqdm(range(len(msg_data['image_msg']))):
            
                # written by anuj
            # bevimage, _ = image_processing.transform_compressed_image(msg_data['image_msg'][i])
            # processed_data['image'].append(bevimage)
            processed_data['image'].append(msg_data['image_msg'][i]) 
            # now find the patches for this image
            curr_odom = msg_data['odom_msg'][i]

            found_patch = False
            for j in range(i, max(i-30, 0), -2):
                prev_image = processed_data['image'][j]
                prev_odom = msg_data['odom_msg'][j]
                # cv2.imshow('src_image', processed_data['src_image'][i])
                # patch, patch_black_pct, curr_img, vis_img = ListenRecordData.get_patch_from_odom_delta(
                patch, patch_black_pct, curr_img, vis_img = image_processing.transformation_from_odom(
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
                # print(f"the original size of the image is {processed_data['image'][i].shape}")
                # msg_data['patches'][i] = [processed_data['image'][i][500:564, 613:677]]
            else:
                found_patch_number += 1
            # remove the i-30th image from RAM
            if i > 30:
                processed_data['image'][i-30] = None

            # was the patch found or no ?
            if found_patch: msg_data['patches_found'][i] = True
            else: msg_data['patches_found'][i] = False

        print(f"no of patches found is {found_patch_number}")
        return msg_data['patches'], msg_data['patches_found']

    #being used the function the process_bev_image_and_patches
    @staticmethod
    def get_patch_from_odom_delta(curr_pos, prev_pos, curr_vel, prev_vel, prev_image, curr_image):
        # print(f"size of image coming in is {prev_image.shape}")
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
        
        CENTER = np.array((500, 95))
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

        ListenRecordData.draw_arrow(mov_start, mov_end, (255, 0, 0), inv_pos_transform, CENTER, vis_img)
        ListenRecordData.draw_arrow(head_start, head_end, (0, 0, 255), inv_pos_transform, CENTER, vis_img)

        projected_loc_prev_frame = inv_pos_transform @ projected_loc_np
        scaled_projected_loc = (projected_loc_prev_frame * 200).astype(np.int64)
        projected_loc_image_frame = CENTER + np.array((-scaled_projected_loc[1], -scaled_projected_loc[0]))
        cv2.circle(vis_img, (projected_loc_image_frame[0], projected_loc_image_frame[1]), 3, (0, 255, 255))

        cv2.imshow('vis_img', vis_img)
        cv2.waitKey(1)
        time.sleep(2)
        #chatgpt bro ko code
        #calculate the scaling factor
        height, width = prev_image.shape[:2]
        scaling_factor = min(height, width) / 64.0

        # Scale the patch corners accordingly
        # scaled_patch_corners_image_frame = [
        #     (center + np.array((-scaled_patch_corners[i][1], -scaled_patch_corners[i][0]))) * scaling_factor for i, center in enumerate(CENTER)
        # ]
        scaled_patch_corners_image_frame = [
            CENTER + np.array((-scaled_patch_corners[0][1], -scaled_patch_corners[0][0])) * scaling_factor,
            CENTER + np.array((-scaled_patch_corners[1][1], -scaled_patch_corners[1][0])) * scaling_factor,
            CENTER + np.array((-scaled_patch_corners[2][1], -scaled_patch_corners[2][0])) * scaling_factor,
            CENTER + np.array((-scaled_patch_corners[3][1], -scaled_patch_corners[3][0])) * scaling_factor
        ]

        # Calculate the scaled patch size
        patch_height, patch_width = int(64 * scaling_factor), int(64 * scaling_factor)

        src_points = np.float32(scaled_patch_corners_image_frame)
        dst_points = np.float32([[0, 0], [patch_width-1, 0], [patch_width-1, patch_height-1], [0, patch_height-1]])
        
        # persp = cv2.getPerspectiveTransform(np.float32(patch_corners_image_frame), np.float32([[0, 0], [63, 0], [63, 63], [0, 63]]))
        # persp = cv2.getPerspectiveTransform(np.float32(scaled_patch_corners_image_frame), np.float32([[0, 0], [patch_width-1, 0], [patch_width-1, patch_height-1], [0, patch_height-1]]))
        persp = cv2.getPerspectiveTransform(np.float32(scaled_patch_corners_image_frame), np.float32([[0, 0], [patch_width-1, 0], [patch_width-1, patch_height-1], [0, patch_height-1]]))
 
        patch = cv2.warpPerspective(prev_image, persp, (patch_width, patch_height))
        # patch = cv2.warpPerspective(
        #     prev_image,
        #     persp,
        #     (64, 64)
        # )

        zero_count = np.logical_and(np.logical_and(patch[:, :, 0] == 0, patch[:, :, 1] == 0), patch[:, :, 2] == 0)

        if np.sum(zero_count) > PATCH_EPSILON:
            # print("Patch is black")
            return None, 1.0, None, None

        return patch, (np.sum(zero_count) / (64. * 64.)), curr_image, vis_img

    #being used in the get_patch_from_odom_delta function
    @staticmethod
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
    
if __name__ == '__main__':
    rospy.init_node('rosbag_data_recorder', anonymous=True)
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-f', '--file', type=str, default='data', help='file to save data to')
    parser.add_argument('-b', '--bag', type=str, default='data/new_at_home.bag', help='rosbag file to read from')
    args = parser.parse_args()
    save_data_path = args.file

    print('save_data_path: ', save_data_path)
    os.makedirs(save_data_path, exist_ok=True)

    if not os.path.exists(args.bag):
        cprint(args.bag, 'red', attrs=['bold'])
        raise FileNotFoundError('ROS bag file not found')

    # start a subprocess to run the rosbag
    rosbag_play_process = subprocess.Popen(['rosbag', 'play', args.bag, '-r', '1'])

    data_recorder = ListenRecordData(save_data_path=save_data_path,
                                     rosbag_play_process=rosbag_play_process)

    while not rospy.is_shutdown():
        # check if python subprocess is still running
        if rosbag_play_process.poll() is not None:
            print('rosbag_play process has stopped')
            data_recorder.save_data(copy.deepcopy(data_recorder.msg_data),
                                    data_recorder.batch_idx + 1)
            exit(0)

    rospy.spin()

