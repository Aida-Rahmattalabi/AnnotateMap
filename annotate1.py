#!/usr/bin/env python


# Every python controller needs this line
import rospy
# The velocity command message
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys; 

from std_msgs.msg import Int32 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import message_filters

import tf 
from tf.transformations import euler_from_quaternion

# mathematical helper functions
import math 
import numpy as np
import time
class turn:
    def __init__(self, threshDis):
        # Set the stopping distance and max speed
        self.distance = threshDis
        self.orientation1 = -1000
	self.orientation2 = 0
	self.vect = []
	self.aligned = False
	self.check_aligned = True
        # A publisher for the move data
        self.pub = rospy.Publisher('/mobile_base/commands/velocity',
                                   Twist,
                                   queue_size=0)

        # A subscriber for the laser data
        #self.sub = rospy.Subscriber('scan', LaserScan, self.callBack)
	pose_sub = message_filters.Subscriber('odom',Odometry)
        scan_sub = message_filters.Subscriber('scan',LaserScan)
        ts = message_filters.ApproximateTimeSynchronizer([pose_sub, scan_sub], 1, 100)
        ts.registerCallback(self.callBack)


 
    def finalize(self):

        rospy.signal_shutdown("User exited turtlebot simulation")
        return True

    def check_alignement(self, pose_data):
    	quaternion = (pose_data.pose.pose.orientation.x,
			pose_data.pose.pose.orientation.y, 
			pose_data.pose.pose.orientation.z, 
			pose_data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	#------------------------------------------------------------
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	#------------------------------------------------------------
	print yaw	
	if yaw < 0:
		yaw = -yaw + math.pi

	#print (yaw/(math.pi/2)) - math.floor((yaw/(math.pi/2)))
	if abs(yaw - round((yaw/(math.pi/2))*(math.pi/2))>=0.06):
		self.aligned = False
	if abs(yaw - round((yaw/(math.pi/2))*(math.pi/2))<0.06):
		self.aligned = True
	
    def align(self, pose_data):
	command = Twist()
	command.linear.x = 0.0
	command.linear.y = 0.0
	command.linear.z = 0.0
	command.angular.x = 0.0
	command.angular.y = 0.0
	if not self.aligned:
		quaternion = (pose_data.pose.pose.orientation.x,
			pose_data.pose.pose.orientation.y, 
			pose_data.pose.pose.orientation.z, 
			pose_data.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
	#------------------------------------------------------------
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
	#-------------------------------------------------------------
		if yaw < 0:
			yaw = yaw + math.pi
		
		
		if abs(yaw - round((yaw/(math.pi/2))*(math.pi/2))>=0.06):
			command.angular.z = 0.1 *np.sign(abs(yaw - round((yaw/(math.pi/2))*(math.pi/2))-0.78))
		    	pub.publish(command) 

		if abs(yaw - round((yaw/(math.pi/2))*(math.pi/2))<0.06):
			self.aligned = True
#-----------------------------------------------------------------------------
    def callBack(self,pose_data,sensor_data):
	
	#self.distance=rospy.get_param('turn')
	if self.check_aligned:
		self.check_alignement(pose_data)
		if not self.aligned:
			self.align(pose_data)
		if self.aligned:
			self.check_aligned = False
	
	self.check_aligned = True
	print self.aligned, self.check_aligned
	if not self.check_aligned:	
		refined_sensor_data = sensor_data.ranges[71:569]
		quaternion = (pose_data.pose.pose.orientation.x,
			pose_data.pose.pose.orientation.y, 
			pose_data.pose.pose.orientation.z, 
			pose_data.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		if yaw < 0:
			yaw = yaw + math.pi

		print "came here"
		#print self.aligned, self.check_aligned
		if self.orientation1 == -1000:
			self.orientation1 = yaw
		self.orientation2 = yaw 

		stop = False
		command = Twist()
		command.linear.x = 0.0
		command.linear.y = 0.0
		command.linear.z = 0.0
		command.angular.x = 0.0
		command.angular.y = 0.0
		command.angular.z = 0.3

		delta =  self.orientation2 - self.orientation1
		if delta < 0:
			delta = delta + math.pi
		#print self.orientation1, self.orientation2, delta
		if delta >= math.pi/2.00:
			stop = True
		if stop:
			command.angular.z = 0.0

			#rospy.sleep(10.0)
			stop = False
			self.orientation1 = -1000

			maxIndice = np.argmax(refined_sensor_data)
			print maxIndice, refined_sensor_data[maxIndice]

			if refined_sensor_data[maxIndice] > self.distance :
				self.vect.append(1)
				self.check_aligned = True
			else:
				self.vect.append(0)
				self.check_aligned = True
		#time.sleep(5)
		pub.publish(command) 
		i1 = refined_sensor_data[0]

		for i in refined_sensor_data:
			pass
		if len(self.vect) == 4:
			print self.vect
			self.align(pose_data)
			self.finalize()







		

if __name__ == '__main__':
    	print round(3.99)
	rospy.init_node('annotate_map')
	input_var=3
	st =turn(input_var)
	# A publisher for the move data
	pub = rospy.Publisher('/mobile_base/commands/velocity',
	          Twist,
	          queue_size=10)
	
	rospy.spin()#!/usr/bin/env python


# Every python controller needs this line
import rospy
# The velocity command message
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys; 

from std_msgs.msg import Int32 
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import message_filters

import tf 
from tf.transformations import euler_from_quaternion

# mathematical helper functions
import math 
import numpy as np
import time
class turn:
    def __init__(self, threshDis):
        # Set the stopping distance and max speed
        self.distance = threshDis
        self.orientation1 = -1000
	self.orientation2 = 0
	self.vect = []
	self.aligned = False
	self.check_aligned = True
        # A publisher for the move data
        self.pub = rospy.Publisher('/mobile_base/commands/velocity',
                                   Twist,
                                   queue_size=0)

        # A subscriber for the laser data
        #self.sub = rospy.Subscriber('scan', LaserScan, self.callBack)
	pose_sub = message_filters.Subscriber('odom',Odometry)
        scan_sub = message_filters.Subscriber('scan',LaserScan)
        ts = message_filters.ApproximateTimeSynchronizer([pose_sub, scan_sub], 1, 100)
        ts.registerCallback(self.callBack)


 
    def finalize(self):

        rospy.signal_shutdown("User exited turtlebot simulation")
        return True

    def check_alignement(self, pose_data):
    	quaternion = (pose_data.pose.pose.orientation.x,
			pose_data.pose.pose.orientation.y, 
			pose_data.pose.pose.orientation.z, 
			pose_data.pose.pose.orientation.w)
	euler = tf.transformations.euler_from_quaternion(quaternion)
	#------------------------------------------------------------
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	#------------------------------------------------------------
	print yaw	
	if yaw < 0:
		yaw = -yaw + math.pi

	#print (yaw/(math.pi/2)) - math.floor((yaw/(math.pi/2)))
	if abs(yaw - round((yaw/(math.pi/2))*(math.pi/2))>=0.06):
		self.aligned = False
	if abs(yaw - round((yaw/(math.pi/2))*(math.pi/2))<0.06):
		self.aligned = True
	
    def align(self, pose_data):
	command = Twist()
	command.linear.x = 0.0
	command.linear.y = 0.0
	command.linear.z = 0.0
	command.angular.x = 0.0
	command.angular.y = 0.0
	if not self.aligned:
		quaternion = (pose_data.pose.pose.orientation.x,
			pose_data.pose.pose.orientation.y, 
			pose_data.pose.pose.orientation.z, 
			pose_data.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
	#------------------------------------------------------------
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
	#-------------------------------------------------------------
		if yaw < 0:
			yaw = yaw + math.pi
		
		
		if abs(yaw - round((yaw/(math.pi/2))*(math.pi/2))>=0.06):
			command.angular.z = 0.1 *np.sign(abs(yaw - round((yaw/(math.pi/2))*(math.pi/2))-0.78))
		    	pub.publish(command) 

		if abs(yaw - round((yaw/(math.pi/2))*(math.pi/2))<0.06):
			self.aligned = True
#-----------------------------------------------------------------------------
    def callBack(self,pose_data,sensor_data):
	
	#self.distance=rospy.get_param('turn')
	if self.check_aligned:
		self.check_alignement(pose_data)
		if not self.aligned:
			self.align(pose_data)
		if self.aligned:
			self.check_aligned = False
	
	self.check_aligned = True
	print self.aligned, self.check_aligned
	if not self.check_aligned:	
		refined_sensor_data = sensor_data.ranges[71:569]
		quaternion = (pose_data.pose.pose.orientation.x,
			pose_data.pose.pose.orientation.y, 
			pose_data.pose.pose.orientation.z, 
			pose_data.pose.pose.orientation.w)
		euler = tf.transformations.euler_from_quaternion(quaternion)
		roll = euler[0]
		pitch = euler[1]
		yaw = euler[2]
		if yaw < 0:
			yaw = yaw + math.pi

		print "came here"
		#print self.aligned, self.check_aligned
		if self.orientation1 == -1000:
			self.orientation1 = yaw
		self.orientation2 = yaw 

		stop = False
		command = Twist()
		command.linear.x = 0.0
		command.linear.y = 0.0
		command.linear.z = 0.0
		command.angular.x = 0.0
		command.angular.y = 0.0
		command.angular.z = 0.3

		delta =  self.orientation2 - self.orientation1
		if delta < 0:
			delta = delta + math.pi
		#print self.orientation1, self.orientation2, delta
		if delta >= math.pi/2.00:
			stop = True
		if stop:
			command.angular.z = 0.0

			#rospy.sleep(10.0)
			stop = False
			self.orientation1 = -1000

			maxIndice = np.argmax(refined_sensor_data)
			print maxIndice, refined_sensor_data[maxIndice]

			if refined_sensor_data[maxIndice] > self.distance :
				self.vect.append(1)
				self.check_aligned = True
			else:
				self.vect.append(0)
				self.check_aligned = True
		#time.sleep(5)
		pub.publish(command) 
		i1 = refined_sensor_data[0]

		for i in refined_sensor_data:
			pass
		if len(self.vect) == 4:
			print self.vect
			self.align(pose_data)
			self.finalize()







		

if __name__ == '__main__':
    	print round(3.99)
	rospy.init_node('annotate_map')
	input_var=3
	st =turn(input_var)
	# A publisher for the move data
	pub = rospy.Publisher('/mobile_base/commands/velocity',
	          Twist,
	          queue_size=10)
	
	rospy.spin()
