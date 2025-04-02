#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

def joy_callback(data):
    # Map joystick axes to linear and angular velocities
    linear_vel = data.axes[1]
    angular_vel = data.axes[3]

    # Create a Twist message and set the linear and angular velocities
    twist = Twist()
    twist.linear.x = linear_vel
    twist.angular.z = angular_vel

    # Publish the Twist message to the /cmd_vel topic
    cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('joy_to_cmd_vel')

    # Create a publisher for the /cmd_vel topic
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Subscribe to the /joy topic and set the callback function
    joy_sub = rospy.Subscriber('/joy', Joy, joy_callback)

    # Spin until the node is stopped
    rospy.spin()

