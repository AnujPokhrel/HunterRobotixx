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
import matplotlib.pyplot as plt
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

PATCH_SIZE = 64
PATCH_EPSILON = 0.5 * PATCH_SIZE * PATCH_SIZE

# TODO -very very important, need this figure this shiz out  
ACTUATION_LATENCY = 0.25

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

        for i in tqdm(range(len(msg_data['image_msg']))):
            
                # written by anuj
            bevimage, _ = image_processing.transform_compressed_image(msg_data['image_msg'][i])
            # bevimage, _ = ListenRecordData.camera_imu_homography(msg_data['imu_msg'][i], msg_data['image_msg'][i])
            processed_data['image'].append(bevimage)

            # cv2.imshow('disp', bevimage)
            # cv2.waitKey(1)

            #save this one image msg and the vector nav msg as a pickle file

            # pickle.dump(msg_data['image_msg'][i], open('/home/haresh/PycharmProjects/visual_IKD/tmp/image_msg.pkl', 'wb'))
            # pickle.dump(msg_data['imu_msg'][i], open('/home/haresh/PycharmProjects/visual_IKD/tmp/imu_msg.pkl', 'wb'))
            # input()

            # now find the patches for this image
            curr_odom = msg_data['odom_msg'][i]

            found_patch = False
            for j in range(i, max(i-30, 0), -2):
                prev_image = processed_data['image'][j]
                prev_odom = msg_data['odom_msg'][j]
                # cv2.imshow('src_image', processed_data['src_image'][i])
                patch, patch_black_pct, curr_img, vis_img = ListenRecordData.get_patch_from_odom_delta(
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

    #not being used anywhere
    #def process_bev_image(self, data):
    #   bevimages = []
    #    for i in tqdm(range(len(data['image_msg']))):
    #        bevimage, _ = ListenRecordData.camera_imu_homography(data['imu_msg'][i], data['image_msg'][i])
    #        bevimages.append(bevimage)
    #    return bevimages

    #@staticmethod
    #def process_patches(data, processed_data):
    #    patches = {}
    #    for i in range(len(processed_data['image'])):
    #        curr_odom = data['odom_msg'][i]
    #        found_patch = False
    #        for j in range(i, max(i - 30, 0), -2):
    #            prev_image = processed_data['image'][j]
    #            prev_odom = data['odom_msg'][j]
    #            # cv2.imshow('src_image', processed_data['src_image'][i])
    #            patch, patch_black_pct, curr_img, vis_img = ListenRecordData.get_patch_from_odom_delta(curr_odom.pose.pose, prev_odom.pose.pose, curr_odom.twist.twist,
    #                                                                                                   prev_odom.twist.twist, prev_image, processed_data['image'][i])
    #            if patch is not None:
    #                found_patch = True
    #                if i not in patches:
    #                    patches[i] = []
    #                patches[i].append(patch)
    #        if not found_patch:
    #            print("Unable to find patch for idx: ", i)
    #        else:
    #            # show the image and the patches
    #            cv2.imshow('image', processed_data['image'][i])
    #            for patch in patches[i]:
    #                cv2.imshow('patch', patch)
    #                cv2.waitKey(0)
    #
    #    return patches


    #being used the function the process_bev_image_and_patches
    @staticmethod
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

        ListenRecordData.draw_arrow(mov_start, mov_end, (255, 0, 0), inv_pos_transform, CENTER, vis_img)
        ListenRecordData.draw_arrow(head_start, head_end, (0, 0, 255), inv_pos_transform, CENTER, vis_img)

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

    #being used in the save data function, will probably need it
    @staticmethod
    def process_joystick_data(data, config):
        # process joystick
        last_speed = 0.0
        joystick_data = []
        for i in range(len(data['joystick_msg'])):
            datum = data['joystick_msg'][i].axes # TODO use a previous joystick command based on actuation LATENCY
            # print(data['joystick'][i])
            steer_joystick = -datum[0]
            drive_joystick = -datum[4]
            turbo_mode = datum[2] >= 0.9
            max_speed = turbo_mode * config['turbo_speed'] + (1 - turbo_mode) * config['normal_speed']
            speed = drive_joystick * max_speed
            steering_angle = steer_joystick * config['maxTurnRate']

            smooth_speed = max(speed, last_speed - config['commandInterval'] * config['accel_limit'])
            smooth_speed = min(smooth_speed, last_speed + config['commandInterval'] * config['accel_limit'])
            last_speed = smooth_speed
            erpm = config['speed_to_erpm_gain'] * smooth_speed + config['speed_to_erpm_offset']
            erpm_clipped = min(max(erpm, -config['erpm_speed_limit']), config['erpm_speed_limit'])
            clipped_speed = (erpm_clipped - config['speed_to_erpm_offset']) / config['speed_to_erpm_gain']

            servo = config['steering_to_servo_gain'] * steering_angle + config['steering_to_servo_offset']
            clipped_servo = min(max(servo, config['servo_min']), config['servo_max'])
            steering_angle = (clipped_servo - config['steering_to_servo_offset']) / config['steering_to_servo_gain']
            rot_vel = clipped_speed / config['wheelbase'] * np.tan(steering_angle)

            datum = [clipped_speed, rot_vel]
            joystick_data.append(datum)

        return joystick_data

    #definately I will not use this function
    # @staticmethod
    # def homography_camera_displacement(R1, R2, t1, t2, n1):
    #     R12 = R2 @ R1.T
    #     t12 = R2 @ (- R1.T @ t1) + t2
    #     # d is distance from plane to t1.
    #     d = np.linalg.norm(n1.dot(t1.T))

    #     H12 = R12 - ((t12 @ n1.T) / d)
    #     H12 /= H12[2, 2]
    #     return H12

    #definately not using this function
    # @staticmethod
    # def camera_imu_homography(imu, image):
    #     orientation_quat = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]
    #     C_i = np.array(
    #         [622.0649233612024, 0.0, 633.1717569157071, 0.0, 619.7990184421728, 368.0688607187958, 0.0, 0.0, 1.0]).reshape(
    #         (3, 3))

    #     R_imu_world = R.from_quat(orientation_quat)
    #     R_imu_world = R_imu_world.as_euler('xyz', degrees=True)
    #     R_imu_world[0], R_imu_world[1] = -R_imu_world[0], R_imu_world[1]
    #     R_imu_world[2] = 0.

    #     R_imu_world = R_imu_world
    #     R_imu_world = R.from_euler('xyz', R_imu_world, degrees=True)

    #     R_cam_imu = R.from_euler("xyz", [90, -90, 0], degrees=True)
    #     R1 = R_cam_imu * R_imu_world
    #     R1 = R1.as_matrix()

    #     R2 = R.from_euler("xyz", [0, 0, -90], degrees=True).as_matrix()
    #     t1 = R1 @ np.array([0., 0., 0.5]).reshape((3, 1))
    #     t2 = R2 @ np.array([-2.5, -0., 6.0]).reshape((3, 1))
    #     n = np.array([0, 0, 1]).reshape((3, 1))
    #     n1 = R1 @ n

    #     H12 = ListenRecordData.homography_camera_displacement(R1, R2, t1, t2, n1)
    #     homography_matrix = C_i @ H12 @ np.linalg.inv(C_i)
    #     homography_matrix /= homography_matrix[2, 2]

    #     img = np.fromstring(image.data, np.uint8)
    #     img = cv2.imdecode(img, cv2.IMREAD_COLOR)

    #     output = cv2.warpPerspective(img, homography_matrix, (1280, 720))
    #     # flip output horizontally
    #     output = cv2.flip(output, 1)

    #     return output, img

    ###not being used anywhere
    # @staticmethod
    # def process_accel_gyro_data(data):
    #     """
    #     Deprecated
    #     """
    #     accel_data = []
    #     gyro_data = []
    #     for i in range(len(data['accel_msg'])):
    #         accel = data['accel_msg'][i].linear_acceleration
    #         gyro = data['gyro_msg'][i].angular_velocity
    #         accel_data.append(np.asarray([accel.x, accel.y, accel.z]))
    #         gyro_data.append(np.asarray([gyro.x, gyro.y, gyro.z]))
    #     return accel_data, gyro_data

if __name__ == '__main__':
    rospy.init_node('rosbag_data_recorder', anonymous=True)
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-f', '--file', type=str, default='data.pkl', help='file to save data to')
    parser.add_argument('-b', '--bag', type=str, default='data/re_collected.bag', help='rosbag file to read from')
    args = parser.parse_args()
    # main(args.file)    
    # config_path = rospy.get_param('config_path')
    # rosbag_path = rospy.get_param('rosbag_path')
    save_data_path = args.file

    # print('config_path: ', config_path)
    # print('rosbag_path: ', rosbag_path)
    #replaced haresh by anuj
    if not args.file:
        save_data_path = args.file.replace('.bag', '_data.pkl')
    
    # if not save_data_path:
    #     save_data_path = rosbag_path.replace('.bag', '_data.pkl')
    
    print('save_data_path: ', save_data_path)
    os.makedirs(save_data_path, exist_ok=True)

    # if not os.path.exists(config_path):
    #     raise FileNotFoundError('Config file not found')
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

