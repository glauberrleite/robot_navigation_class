#!/usr/bin/env python2

import rospy
import sys
import numpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class Mover:

	def __init__(self, name):
		rospy.init_node('mover_{}'.format(name), anonymous=True)
		self.rate = rospy.Rate(5)
		self.tolerance = 0.1
		self.oldCost = float('Inf')
		
		self.current = numpy.array([0, 0, 0])

		rospy.Subscriber('odom', Odometry, self.odomCallback)
		rospy.Subscriber('base_scan', LaserScan, self.scanCallback)
		
		self.vel = rospy.Publisher('cmd_vel', Twist)

		'Giving a time for the subscribed topics to give the first messages'
		self.rate.sleep()

	def odomCallback(self, msg):
		self.current = numpy.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
		self.theta = 2 * numpy.arccos(msg.pose.pose.orientation.w) * numpy.sign(msg.pose.pose.orientation.z)
	
	def scanCallback(self, msg):
		self.scan = msg
	
	def sensorIndex (self, angle):
		return int((angle - self.scan.angle_min)/self.scan.angle_increment)
	
	def sensorAngle(self, index):
		return float(index * self.scan.angle_increment + self.scan.angle_min)

	def cost(self, index):
		result = float('Inf')
		threshold = self.scan.range_max
		
		d_xn = self.scan.ranges[index]
		
		if d_xn >= threshold:
			
			delta = numpy.array([d_xn * numpy.cos(self.sensorAngle(index) + self.theta), d_xn * numpy.sin(self.sensorAngle(index) + self.theta)])
			q_n = self.current + delta
			d_nq = numpy.linalg.norm(self.goal - q_n)
			
			result = d_xn + d_nq
			
		return result

	def avoidCollisions(self, angle):
		deltaAngle = 20 * numpy.pi / 180

		zeroAngleIndex = self.sensorIndex(0)
		clockWiseRot = False
		tooClose = False

		minRange = self.scan.range_max / 2
		for i in range(len(self.scan.ranges)):
			if self.scan.ranges[i] < minRange:
				tooClose = True
				if i < self.sensorIndex(0):
					clockWiseRot = True
				else:
					clockWiseRot = False
					break

		if tooClose:
			if clockWiseRot:
				angle = angle + deltaAngle
			else:
				angle = angle - deltaAngle

		return angle

	def goTo(self, goal, gain):
		self.goal = goal

		error = goal - self.current
		error_norm = numpy.linalg.norm(error)
		
		while error_norm >= self.tolerance:
			print(self.current)

			angle = 0
			cost = float('Inf')
			avoidCollisions = True

			'If path is locally clean for q_goal, go that direction'
			angleToGoal = numpy.arctan2(error[1], error[0]) - self.theta
			rangeToGoal = self.scan.ranges[numpy.minimum(self.sensorIndex(angleToGoal), len(self.scan.ranges) - 1)]
		
			if (rangeToGoal >= self.scan.range_max) or (error_norm <= rangeToGoal):
				angle = angleToGoal
				cost = self.oldCost
				
				if (error_norm <= rangeToGoal):
					avoidCollisions = False
			else:
				'If we met an obstacle to goal, we need to find another direction'
				minCost = float('Inf')

				for i in range(len(self.scan.ranges)):
					cost = self.cost(i)
					
					if cost < minCost:
						minCost = cost
						angle = self.sensorAngle(i)
			
			if avoidCollisions:
				angle = self.avoidCollisions(angle)
	
			if (cost > self.oldCost):
				print('We have a situation here')
			self.oldCost = cost

			'Move using computed angle'
			vel_msg = Twist()

			vel_msg.linear.x = gain
			vel_msg.linear.y = 0
			vel_msg.linear.z = 0
			vel_msg.angular.x = 0
			vel_msg.angular.y = 0
			vel_msg.angular.z = angle
			
			self.vel.publish(vel_msg)

			self.rate.sleep()
			
			error = goal - self.current
			error_norm = numpy.linalg.norm(error)

kobuki = Mover('kobuki')
goal = numpy.array([float(sys.argv[1]), float(sys.argv[2])])
gain = float(sys.argv[3])

kobuki.goTo(goal, gain)
