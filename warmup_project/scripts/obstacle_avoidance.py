#!/usr/bin/env python

""" Have bot avoid obstacles while traversing from Point A to Point B. """

import rospy
from geometry_msgs.msg import Twist, PoseWithCovariance, Pose, Point
from sensor_msgs.msg import LaserScan, Image
import math

class Node:
	def __init__(self):
		self.x =
		self.y =

class Controller:
	def __init__(self):
		rospy.init_node('person_stop')
		rospy.Subscriber('/scan', LaserScan, self.read_scan, queue_size=1)
		rospy.Subscriber('/odom', Odometry, self.read_odom, queue_size=1)
		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

		self.command = Twist()
		self.stop()

	def charge(self,scan,odom):
		''' if path in front of bot is passable, move forward.  if not, find which node is passable '''

	def locate_obstacle(self,scan,odom):
		''' find the obstacle's location and map it to a node '''
	
		# NONE OF THE SYNTAX IS CURRENTLY CORRECT

		# bot's location as per odom
		bot_x = odom.x
		bot_y = odom.y

		# obstacle's cylindrical location as per scan
		theta = scan_index
		radius = scan_reading

		# obstacle's cartesian location in relation to base_link
		delta_x = dist*math.cos(math.radians(theta))
		delta_y = dist*math.sin(math.radians(theta))

		# obstacle's location in relation to world origin
		obst_x = bot_x + delta_x
		obst_y = bot_y + delta_y

		return [obst_x, obst_y]

	def construct_map(self):
		''' this is done when the bot is first turned on to get a sense of the immediate field '''

	def read_scan(self,scan):
		# ''' read the scan from 310-365, 0-45 '''
		# measure_left = scan.ranges[45:10:-1]
		# measure_right = scan.ranges[350:315:-1]

		# detect_left = sum([i for i in measure_left if i>0 and i<1.5])
		# detect_right = sum([i for i in measure_right if i>0 and i<1.5])		
		
		# if (abs(detect_left - detect_right)) < 5:
		# 	print 'go fwd'
		# 	self.forward(0.1)
		# elif detect_left > detect_right:
		# 	print 'spin left'
		# 	self.spin_left(0.3)
		# elif detect_right > detect_left:
		# 	print 'spin right'
		# 	self.spin_right(0.3)
		# else:
		# 	print 'stop'
		# 	self.stop()

	def read_odom(self,odom):

	### Manipulating bot

	def forward(self,speed):
		''' drive bot forward '''
		self.command.linear.x = speed
		self.command.linear.y = 0
		self.command.linear.z = 0
		self.command.angular.x = 0
		self.command.angular.y = 0	
		self.command.angular.z = 0	

	def spin_left(self,speed):
		''' spin bot left '''
		self.command.linear.x = 0
		self.command.linear.y = 0
		self.command.linear.z = 0
		self.command.angular.x = 0
		self.command.angular.y = 0	
		self.command.angular.z = speed			

	def spin_right(self,speed):
		''' spin bot right '''
		self.command.linear.x = 0
		self.command.linear.y = 0
		self.command.linear.z = 0
		self.command.angular.x = 0
		self.command.angular.y = 0	
		self.command.angular.z = -speed	

	def stop(self):
		''' stop all bot motion '''
		self.command.linear.x = 0
		self.pub.publish(self.command)

	def drive(self):
		self.pub.publish(self.command)

controller = Controller()

while not rospy.is_shutdown():
	controller.drive()