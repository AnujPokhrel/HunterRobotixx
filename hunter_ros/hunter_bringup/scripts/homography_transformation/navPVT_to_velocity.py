#!/usr/bin/env python

import rospy
from ublox_msgs.msg import NavPVT
from geometry_msgs.msg import TwistWithCovarianceStamped
import math

def NE_velocity_to_Robot_x_y_vel(vel_N, vel_E, heading):
    theta = math.radians(heading/10000)
    Vx = vel_N*math.cos(theta) + vel_E*math.sin(theta)
    Vy = -vel_N*math.sin(theta) + vel_E*math.cos(theta)
    return [Vx, Vy]


def pvt_callback(data):
    #get data from navpvt and convert it into velocity
    #just a reminder that the velocities will be in the robot frame
    twist = TwistWithCovarianceStamped()
    twist.header.stamp = rospy.Time.now()
    twist.header.frame_id = "base_link"

    converted_vel = NE_velocity_to_Robot_x_y_vel(data.velN * 1e-3, data.velE * 1e-3, data.heading)

    twist.twist.twist.linear.x = converted_vel[0]
    twist.twist.twist.linear.y = converted_vel[1] 
    twist.twist.twist.linear.z = -data.velD * 1e-3
    
    covSpeed = math.pow(data.sAcc * 1e-3, 2)
    cols = 6
    twist.twist.covariance[cols*0 + 0] = covSpeed
    twist.twist.covariance[cols*1 + 1] = covSpeed
    twist.twist.covariance[cols*2 + 2] = covSpeed
    twist.twist.covariance[cols*3 + 3] = -1

    # Publish the Twist message to the /calculated_velocity topic
    calculated_velocity_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('navPVT_to_velocity')

    # Create a publisher for the /calculated_velocity topic
    calculated_velocity_pub = rospy.Publisher('/calculated_velocity', TwistWithCovarianceStamped, queue_size=10)

    # Subscribe to the /f9p_rover/navpvt topic and set the callback function
    pvt_sub = rospy.Subscriber('/f9p_rover/navpvt', NavPVT, pvt_callback)

    # Spin until the node is stopped
    rospy.spin()

