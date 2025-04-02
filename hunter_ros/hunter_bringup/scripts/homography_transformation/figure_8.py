#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import sin, pi

def smooth_move_robot():
    pub = rospy.Publisher('/husky/husky_velocity_controller/cmd_vel', Twist, queue_size=10)
    rospy.init_node('smooth_figure_eight', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # Create Twist messages to represent the forward motion and turn
    move_forward = Twist()
    turn = Twist()

    t = 0.0 # Current time
    dt = 0.1 # Time step, corresponding to the rate
    period = 30.0 # Period of one full figure eight

    while not rospy.is_shutdown():
        fraction = (t % period) / period # Fraction of the period that has passed

        if fraction < 0.25:
            # First half of the first circle
            move_forward.linear.x = 2.0
            turn.angular.z = 2.0 * pi / period
        elif fraction < 0.5:
            # Second half of the first circle
            move_forward.linear.x = 2.0
            turn.angular.z = -2.0 * pi / period
        elif fraction < 0.75:
            # First half of the second circle
            move_forward.linear.x = 2.0 
            turn.angular.z = -2.0 * pi / period
        else:
            # Second half of the second circle
            move_forward.linear.x = 2.0
            turn.angular.z = 2.0 * pi / period

        velocity = Twist()
        velocity.linear.x = move_forward.linear.x
        velocity.angular.z = turn.angular.z
        pub.publish(velocity)

        t += dt
        rate.sleep()

if __name__ == '__main__':
    try:
        smooth_move_robot()
    except rospy.ROSInterruptException:
        pass
