#this script just plays back the pickle file that was saved in bag_2_pkl.py

import pickle
import rospy
import cv2
from cv_bridge import CvBridge
import time
import os

rospy.init_node('replay_node')

# Initialize CvBridge
bridge = CvBridge()

data = {}
with open('data/data_1.pkl', 'rb') as file:
    data = pickle.load(file)
    print("type of data is", type(data))
    # print("size of the file is ", os.path.getsize('1st_try_JUL18.pkl'))
    # print("length of data is", len(data))
    print(f"no of data in patches is {len(data['patches'])}")
    print(f"no of data in patches found is {len(data['patches_found'])}")
    print(f"no of data in odom_1sec_msg is {len(data['odom_1sec_msg'])}")
    print(f"no of data in velocity_msg is {len(data['velocity_msg'])}")
    print(f"no of data in cmd_vel_msg is {len(data['cmd_vel_msg'])}")
    print(f"no of data in imu_msg is {len(data['imu_msg'])}")

# for each in data['patches']:
#     # print(data[each])
#     # print(f"type of each is {type(each)}")
#     # print(f"each is {each[0]} and size is {each[0].shape}")
#     #show image if each[0] is not a blank array
#     if each[0].any():
#         cv2.imshow('disp', each[0])
#         cv2.waitKey(1)
#     else:
#         print("blank array")
#     # cv2.imshow('disp', each)
#     # cv2.waitKey(1)
#     # print(f"Time:, {each}, velocity X was, {data[each]['velocity_msg']} and cmd_vel was {data[each]['cmd_vel'].twist.linear}")
#     time.sleep(0.2)

for i in range(len(data['patches'])):
    print("\n\n\n")
    print(f"for the {i}th index")
    # print(f" no of patches in the {i}th index is  {len(data['patches'][i])}")
    for index, each in enumerate(data['patches'][i]):
        if each.any():
            cv2.imshow('disp', each)
            cv2.waitKey(1)
            print(f"inner index is {index}")
            time.sleep(1)
        else:
            print("blank array")
    
