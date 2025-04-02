#this one reprocesses the pickle file from time_trial.py to get the velocity at the end of the window
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
import argparse

def read_file(input_file):
    with open(input_file, 'rb') as file:
        data = pickle.load(file)
    
    keys = sorted(data.keys())
    keys_np = np.array(keys)
    return data, keys_np

def find_nearest_key(keys_np, new_timing):
    keys_float_np = keys_np.astype(np.float64)
    #find the index of the nearest key
    idx = (np.abs(keys_float_np - new_timing)).argmin()
    nearest_key = keys_np[idx]
    return nearest_key     

#a function to solve a quadratic equation
def solving_quadratic(a, b, c):
    #calculate the discriminant
    d = (b**2) - (4*a*c)
    #find two solutions
    sol1 = (-b-m.sqrt(d))/(2*a)
    sol2 = (-b+m.sqrt(d))/(2*a)
    print(f"The solutions are {sol1} and {sol2}")
    return [sol1, sol2]

#function to convert N_velocity and East_velocity to v and omega
def NE_velocity_to_Robot_x_y_vel(vel_N, vel_E, heading):
    theta = m.radians(heading/10000)
    Vx = vel_N*m.cos(theta) + vel_E*m.sin(theta)
    Vy = -vel_N*m.sin(theta) + vel_E*m.cos(theta)
    return [Vx, Vy]


def main(input_file, output_file):
    #read file into data and get the kyes of the data
    print("Reading file")
    data, keys_np = read_file(input_file)
    new_data_dump = {}

    file_to_write = open(output_file, 'ab') 
    
    for each in keys_np:
        actual_vel_inside = 0.0
        actual_vel_outside = 0.0
        vel_going_in = 0.0
        hero_vel = 0.0


        #1.27, 0.885 and 3.5 are the distances, these are hard coded, need to re-visit with new distances.
        # or maybe dynamic distances, if we change the window to change as well.
        # think later  
        #           'bev_image': bev_image,
        #           'cmd_vel': self.recent_data[2],
        #           'velocity_msg': self.recent_data[0],
        #           'imu': self.recent_data[3]
        

        hero_vel = NE_velocity_to_v_w(data[each]['velocity_msg'].twist.twist.linear.x, data[each]['velocity_msg'].twist.twist.linear.y)[0]
        hero_accln = data[each]['imu'].linear_acceleration.y
        #implement newton's laws of motion here to find the elapsed time. just distance/velocity isn't enough
        if hero_vel >= 0.2:
            possible_times = solving_quadratic(0.5*hero_accln, hero_vel, -1.27)
            elapsed_time = max(possible_times)
            print(f"each here is {each}")
            print(f"elapsed_time in the first step {elapsed_time}")
            # elapsed_time = 1.27/hero_vel

            new_timing = float(each) + elapsed_time
            
            #find the index of the nearest key
            nearest_key = find_nearest_key(keys_np, new_timing)
            print(f"nearest_key is {nearest_key}")
            data_at_time = data[nearest_key]
            vel_going_in = NE_velocity_to_v_w(data_at_time['velocity_msg'].twist.twist.linear.x, data_at_time['velocity_msg'].twist.twist.linear.y)[0]
            # vel_going_in = data_at_time['velocity_msg'].linear_velocity
            accln_going_in = data_at_time['imu'].linear_acceleration

        if vel_going_in != 0.0:
            possible_times_inside = solving_quadratic(0.5*accln_going_in, vel_going_in, -0.885)
            elapsed_time_inside = max(possible_times_inside)
            # elapsed_time_inside = 0.885/vel_going_in
            new_timing_inside = float(nearest_key) + elapsed_time_inside

            #find the index of the nearest key
            nearest_key_inside = find_nearest_key(keys_np, new_timing_inside)
            data_at_time_inside = data[nearest_key_inside]

            #change this to cmd_vel
            vel_instruction_inside = data_at_time_inside['cmd_vel'].twist.twist.linear.x
            # actual_vel_inside = data_at_time_inside['velocity_msg'].linear_velocity
            actual_vel_inside = NE_velocity_to_v_w(data_at_time_inside['velocity_msg'].twist.twist.linear.x, data_at_time_inside['velocity_msg'].twist.twist.linear.y)[0]
            accln_inside = data_at_time_inside['imu'].linear_acceleration

        if actual_vel_inside != 0.0:
            possible_times_outside = solving_quadratic(0.5*accln_inside, actual_vel_inside, -3.5)
            elapsed_time_outside = max(possible_times_outside)
            # elapsed_time_outside = 3.5/actual_vel_inside
            new_timing_outside = float(nearest_key_inside) + elapsed_time_outside

            #find the index of the nearest key
            nearest_key_outside = find_nearest_key(keys_np, new_timing_outside)
            data_at_time_outside = data[nearest_key_outside]

            # actual_vel_outside = data_at_time_outside['velocity_msg'].linear_velocity
            actual_vel_outside = NE_velocity_to_v_w(data_at_time_outside['velocity_msg'].twist.twist.linear.x, data_at_time_outside['velocity_msg'].twist.twist.linear.y)[0]
            accln_outside = data_at_time_outside['imu'].linear_acceleration

        if actual_vel_inside != 0.0 and actual_vel_outside != 0.0 and vel_going_in != 0:

            print("writing ordered to dict")
            new_data_dump[each] = {
                'u_in': vel_going_in,
                'a_going_in': accln_going_in,
                'cmd_vel': vel_instruction_inside,
                'u_inside': actual_vel_inside,
                'a_in': accln_inside,
                'u_out': actual_vel_outside,
                'bev_image': data[each]['bev_image']
            }
    
    print("dumping data to file")
    pickle.dump(new_data_dump, file_to_write)
    file_to_write.close()
    print("dump complete")
            
if __name__ == "__main__":
    parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument('-i', '--input', type=str, default='outdoor_data_1st_try.pkl', help='file to read from')
    parser.add_argument('-o', '--output', type=str, default='data_otpt.pkl', help='file to write to')
    args = parser.parse_args()
    print(f"input file: {args.input} and output file: {args.output}")
    main(args.input, args.output)