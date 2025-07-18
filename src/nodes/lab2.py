#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Bool


from tf2_ros.transform_listener import TransformListener   
from tf2_ros.buffer import Buffer


import math

class Lab2:

	def __init__(self):
		"""
		Class constructor
		"""
		self.px = 0.0
		self.py = 0.0
		self.pth = 0.0

		### REQUIRED CREDIT
		### Initialize node, name it 'lab2'
		rospy.init_node('lab2')

		### Tell ROS that this node publishes Twist messages on the '/cmd_vel' topic
		self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		### Tell ROS that this node subscribes to Odometry messages on the '/odom' topic
		### When a message is received, call self.update_odometry
		# rospy.Subscriber('/odom', Odometry, self.update_odometry)
		rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_odometry)
		### Tell ROS that this node subscribes to PoseStamped messages on the '/move_base_simple/goal' topic
		### When a message is received, call self.go_to
		#rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.go_to)
		#pass delete this when you implement your code
		rospy.Subscriber('/path_planner/path', Path, self.go_to)

		self.motion_pub = rospy.Publisher('/move_check', Bool, queue_size=10)


	def send_speed(self, linear_speed, angular_speed):
		"""
		Sends the speeds to the motors.
		:param linear_speed  [float] [m/s]   The forward linear speed.
		:param angular_speed [float] [rad/s] The angular speed for rotating around the body center.
		"""
		### REQUIRED CREDIT
		### Make a new Twist message
		msg_cmd_vel = Twist()
		# Linear velocity
		msg_cmd_vel.linear.x = linear_speed
		msg_cmd_vel.linear.y = 0.0
		msg_cmd_vel.linear.z = 0.0
		# Angular velocityodom
		msg_cmd_vel.angular.x = 0.0
		msg_cmd_vel.angular.y = 0.0
		msg_cmd_vel.angular.z = angular_speed
		### Publish the message
		self.pub.publish(msg_cmd_vel)
		#pass delete this when you implement your code

	
		
	def drive(self, distance, linear_speed):
		"""
		Drives the robot in a straight line.
		:param distance	 [float] [m]   The distance to cover.
		:param linear_speed [float] [m/s] The forward linear speed.
		"""
		### REQUIRED CREDIT
		#record initial x and y coordinate
		pre_pose_x = self.px
		pre_pose_y = self.py
		cur_dis = 0.0
		#drive according to the linear speed given
		self.send_speed(linear_speed,0.0)
		#stop when we have drived to desired location
		while (cur_dis+0.15*distance) <= distance:
			cur_dis = math.sqrt((pre_pose_x-self.px)**2 + (pre_pose_y - self.py)**2)
			rospy.sleep(0.001)
		
		self.send_speed(0.0,0.0)



	def rotate(self, angle, aspeed):
		"""
		Rotates the robot around the body center by the given angle.
		:param angle		 [float] [rad]   The distance to cover.
		:param angular_speed [float] [rad/s] The angular speed.
		"""
		### REQUIRED CREDIT
		#record initial angle according to x-axis
		pre_pose = self.pth
		cur_an = 0.0
		# turn according to the angular speed given
		self.send_speed(0.0, aspeed)
		#stop when we have turned to the desired angle
		while (cur_an+0.08) <= angle:
			cur_an = abs(pre_pose-self.pth)
			rospy.sleep(0.001)
		self.send_speed(0,0)



	def go_to(self, msg):
		"""
		Calls rotate(), drive(), and rotate() to attain a given pose.
		This method is a callback bound to a Subscriber.
		:param msg [PoseStamped] The target pose.
		"""
		### REQUIRED CREDIT
		nav_list = msg.poses

		# print("goto=========")
		# print(nav_list)
		# print("goto end================")


		motion_complete = Bool()
		motion_complete.data = False
		self.motion_pub.publish(motion_complete)
		for anv_cor in nav_list:
			#Everytime record the position user clicked
			goal_X = anv_cor.pose.position.x
			goal_Y = anv_cor.pose.position.y
			#calculate the angled needed to turn to head that goal location
			target_angle = math.atan2(goal_Y-self.py, goal_X-self.px)
			angle_to_turn = target_angle - self.pth
			#we always want to turn least distance. So based on which side the goal is, we turn in different direction
			if(angle_to_turn>=0):
				if angle_to_turn>math.pi:
					self.rotate(abs((2*math.pi)-abs(angle_to_turn)), -0.65)
				else:
					self.rotate(angle_to_turn, 0.95)
			else:
				if angle_to_turn<-math.pi:
					self.rotate(abs((2*math.pi)-abs(angle_to_turn)), 0.65)
				else:
					self.rotate(abs(angle_to_turn), -0.95)
			rospy.sleep(0.05)
			#calculate the distance we need to drive after turning in order to arrive the goal location
			distance_to_go = math.sqrt((goal_X-self.px)**2 + (goal_Y - self.py)**2)
			#drive to goal location after turning
			self.drive(distance_to_go, 0.1)
			rospy.sleep(0.05)
			#calculate the pose after we arrived the goal location
			# quat_orig = anv_cor.pose.orientation
			# quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
			# (roll , pitch , yaw) = euler_from_quaternion(quat_list)
			# final_angle = yaw - self.pth
			# #turn into the final pose which is defined be user.
			# if(angle_to_turn>=0):
			# 	self.rotate(final_angle, 0.5)
			# else:
			# 	self.rotate(abs(final_angle), -0.5)

		# while(True):
		# 	ii = 1


		motion_complete.data = True
		self.motion_pub.publish(motion_complete)


		#extra credits
		#the smooth_drive is implemented correctly
		#self.smooth_drive(1, 0.15)
		#self.arc_to(msg)
		#pass delete this when you implement your code



	def update_odometry(self, msg):
		"""
		Updates the current pose of the robot.
		This method is a callback bound to a Subscriber.
		:param msg [Odometry] The current odometry information.
		"""

		# self._tf_buffer = Buffer()
		# self._tf_listener = TransformListener(self._tf_buffer)
		# trans = self._tf_buffer.lookup_transform("odom", "map", rospy.Time(0))
		# new_pose = self._tf_buffer.transform(msg,"map",rospy.Time(0))


		### REQUIRED CREDIT
		#derive the x and y coordinate from pose message
		self.px = msg.pose.pose.position.x
		self.py = msg.pose.pose.position.y
		quat_orig = msg.pose.pose.orientation
		#derive the angle according to globle x axis converted from the quaternion from pose message
		quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
		(roll , pitch , yaw) = euler_from_quaternion(quat_list)
		self.pth = yaw

		print(msg.pose.pose)
		#pass delete this when you implement your code




	def arc_to(self, position):
		"""
		Drives to a given position in an arc.
		:param msg [PoseStamped] The target pose.
		"""
		### EXTRA CREDIT
		# define a angular velocity (which turns out to be just a scale)
		w = 1
		# remember start point
		X0 = self.px
		Y0 = self.py
		Theta0 = self.pth

		# check target coordiante
		X1 = position.pose.position.x
		Y1 = position.pose.position.y
		quat_orig = position.pose.orientation
		quat_list = [quat_orig.x, quat_orig.y, quat_orig.z, quat_orig.w]
		(roll , pitch , yaw) = euler_from_quaternion(quat_list)
		Theta1 = yaw - self.pth

		# calculate gradient and intersept
		k0 = 1/math.tan(Theta0)
		k1 = 1/math.tan(Theta1)

		b0 = Y0 + k0*X0
		b1 = Y1 + k1*X1

		# calculate the coordinate of the intersect point
		Xi = (b0 - b1)/(k0 - k1)
		Yi = k0*Xi +b0

		# calculate R
		R = math.sqrt((X0 - Xi)**2 + (Y0 - Yi)**2)

		# calculate left and right wheel velocity
		Vr = w*(R + 0.078)
		Vl = w*(R - 0.078)

		# calculate velocity assume the sendspeed function send linear speed plus or minus angular velocity
		LinearSpeed = w*R 
		AngularSpeed = -w*0.078 

		# send velocity
		self.send_speed(LinearSpeed, AngularSpeed)
		Edist = math.sqrt((X1 - self.px)**2 + (Y1 - self.py)**2)
		while (Edist) >= 0.005:
			Edist = math.sqrt((X1 - self.px)**2 + (Y1 - self.py)**2)
			rospy.sleep(0.001)
		self.send_speed(0,0)

		pass # delete this when you implement your code



	def smooth_drive(self, distance, linear_speed):
		"""
		Drives the robot in a straight line by changing the actual speed smoothly.
		:param distance	 [float] [m]   The distance to cover.
		:param linear_speed [float] [m/s] The maximum forward linear speed.
		"""
		### EXTRA CREDIT
		#record initial x and y coordinate
		pre_pose_x = self.px
		pre_pose_y = self.py
		cur_dis = 0.0
		speed = 0.0
		#stop when we have drived to desired location
		while cur_dis <= distance:
			cur_dis = math.sqrt((pre_pose_x-self.px)**2 + (pre_pose_y - self.py)**2)
			# in the begining speed up proportionally to distance to desired speed.
			if(cur_dis <= distance*(1/3)):
				speed = (cur_dis*5/distance)*linear_speed
				if(speed == linear_speed):
					speed = speed
			#before arrive, speed down proportionally to distance to 0
			if(cur_dis >= distance*(2/3)):
				speed = ((distance-cur_dis)/distance)*linear_speed + 0.05
				if(speed <= 0.051):
					speed = 0.00
			self.send_speed(speed,0.0)
			rospy.sleep(0.01)
		#pass # delete this when you implement your code



	def run(self):
		rospy.spin()
	#while not rospy.is_shutdown():

if __name__ == '__main__':
	Lab2().run()
