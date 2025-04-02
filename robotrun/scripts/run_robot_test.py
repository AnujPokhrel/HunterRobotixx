#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import sin, cos, pi
#import yaml
#import torch
#from utils import BEVLidarImage


class ROBOTRUNNER:
    def __init__(self):

        rospy.init_node('node_parser')

        print('Loading the model from checkpoint')

        print('model loaded')

        self.current_pose = None

        #self.config_path= config
        #self.config = yaml.safe_load(open(self.config_path, 'r'))

        # Subscribe to odom topic
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Setup timer to get global plan every 5 seconds
       # self.global_plan_timer = rospy.Timer(rospy.Duration(1), self.get_global_plan)

        # Setup publisher for goal topic
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.path_sub = rospy.Subscriber(
                '/move_base/TrajectoryPlannerROS/global_plan', Path, self.get_global_plan)
        # Initialize global path to None
        self.global_path = None

    def odom_callback(self, msg):
        # Save current pose
        self.current_pose = msg.pose.pose
        self.goal_pub.publish(self.spin())
    def get_global_plan(self,path_msg):
        self.global_path = path_msg
        # Get global plan from move_base node
        try:
           # path_msg = rospy.wait_for_message('/move_base/TrajectoryPlannerROS/global_plan', Path, timeout=1.0)

            # Print the path
            # for pose in path_msg.poses:
            #     print("Pose: ({}, {}, {})".format(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z))
            self.global_path = path_msg
          #  print(self.global_path )
        except rospy.exceptions.ROSException:
            rospy.logwarn('Timed out waiting for global plan')

        print(self.current_pose)
    def spin(self):
#        while not rospy.is_shutdown():

           
            # Calculate future pose in 10 meters and publish goal
            if self.current_pose is not None:
                x = self.current_pose.position.x + 10*cos(euler_from_quaternion([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w])[2])
                y = self.current_pose.position.y + 10*sin(euler_from_quaternion([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w])[2])
                z = self.current_pose.position.z
                qx, qy, qz, qw = quaternion_from_euler(0, 0, euler_from_quaternion([self.current_pose.orientation.x, self.current_pose.orientation.y, self.current_pose.orientation.z, self.current_pose.orientation.w])[2])

                # Publish goal
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = x
                goal.pose.position.y = y
                goal.pose.position.z = z
                goal.pose.orientation.x = qx
                goal.pose.orientation.y = qy
                goal.pose.orientation.z = qz
                goal.pose.orientation.w = qw

                return goal
            # Sleep for a small amount of time
            # rospy.sleep(0.1)

if __name__ == '__main__':
    try:

        robot_node = ROBOTRUNNER()
        rospy.spin()


    except rospy.ROSInterruptException:
        pass
