#!/usr/bin/env python2

import rospy
import sys
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

def callback(msg):
    global current
    current = numpy.array([msg.pose.pose.position.x, msg.pose.pose.position.y])

rospy.init_node('mover')

sub = rospy.Subscriber('odom', Odometry, callback)
pub = rospy.Publisher('cmd_vel', Twist)

goal = numpy.array([float(sys.argv[1]), float(sys.argv[2])])

'Constante de proporcionalidade'
K = float(sys.argv[3])

'Publicar uma vez por segundo'
rate = rospy.Rate(5)
rate.sleep()

'Tolerancia'
epsilon = 0.01
error = epsilon

while error >= epsilon:
    global current
    
    print(current)

    vel = K * (goal - current)

    vel_msg = Twist()

    vel_msg.linear.x = float(vel[0])
    vel_msg.linear.y = float(vel[1])
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    pub.publish(vel_msg)

    error = numpy.linalg.norm(goal - current)

    rate.sleep()

