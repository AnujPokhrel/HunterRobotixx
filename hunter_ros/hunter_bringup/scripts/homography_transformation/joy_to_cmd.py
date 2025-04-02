#!/usr/bin/env python4

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped, TwistStamped

class Joy2CmdVelParams:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', TwistStamped, queue_size=10)
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        #self.twist = Twist()
        self.twist = TwistStamped()
        self.rate = rospy.Rate(10)
        self.max_linear_vel = 4.8

        while not rospy.is_shutdown():
            self.twist.header.stamp = rospy.Time.now()
            self.cmd_vel_pub.publish(self.twist)
            self.rate.sleep() 

        rospy.spin()

    def joy_callback(self, data):
        # Map joystick axes to linear and angular velocities
        linear_vel = data.axes[1]
        angular_vel = data.axes[3]

        # Create a Twist message and set the linear and angular velocities
        # self.twist = Twist()
        self.twist.twist.linear.x = linear_vel * self.max_linear_vel
        self.twist.twist.angular.z = angular_vel

        # Publish the Twist message to the /cmd_vel topic
        print("changing twist")
        # self.cmd_vel_pub.publish(self.twist)

if __name__ == '__main__':
    rospy.init_node('joy_to_cmd_vel')

    joyous = Joy2CmdVelParams()

    # Create a publisher for the /cmd_vel topic
    # cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    # Subscribe to the /joy topic and set the callback function
    # joy_sub = rospy.Subscriber('/joy', Joy, joy_callback)

    # rate = rospy.Rate(10)

    # while not rospy.is_shutdown():
    #     cmd_vel_pub.publish(twist)
    #     rate.sleep()
    # Spin until the node is stopped
    # rospy.spin()

