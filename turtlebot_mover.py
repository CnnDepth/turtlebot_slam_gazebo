#!/usr/bin/env python
# coding=utf-8
import rospy
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose
import math
import tf
import timeit

class TurtlebotMover:

	def __init__(self, 
		         name='turtlebot3',
		         max_angular_speed=1,
		         max_linear_speed=0.5,
		         rate=50
		         ):
		self.model_name = name
		self.get_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
		self.state = ModelState()
		self.state.model_name = self.model_name
		initial_state = self.get_state(self.model_name, '')
		self.state.pose = initial_state.pose
		self.ground_z = self.state.pose.position.z
		self.rate = rate
		self.angular_speed = max_angular_speed
		self.linear_speed = max_linear_speed
		rospy.init_node('move_robot')
		self.state_publisher = rospy.Publisher("/gazebo/set_model_state", ModelState, latch=True)

	def get_position(self):
		return self.state.pose.position

	def get_angular_difference(self, angle1, angle2):
		x1, y1, z1 = angle1
		x2, y2, z2 = angle2
		dx = x2 - x1
		dy = y2 - y1
		dz = z2 - z1
		dx = self.normalize(dx)
		dy = self.normalize(dy)
		dz = self.normalize(dz)
		return (dx, dy, dz)

	def normalize(self, angle):
		angle += 2 * math.pi
		if angle > math.pi:
			angle -= 2 * math.pi
		if angle > math.pi:
			angle -= 2 * math.pi
		return angle

	def rotate_to(self, target_angle):
		# get current orientation
		orientation = self.state.pose.orientation
		current_euler_angles = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
		cur_x, cur_y, cur_z = current_euler_angles
		# calculate rotation between current and target orientation
		target_euler_angles = (0, 0, target_angle)
		trg_x, trg_y, trg_z = target_euler_angles
		dx, dy, dz = self.get_angular_difference(current_euler_angles, target_euler_angles)
		# rotate robot with speed of self.angular_speed
		rotation_time = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2) / self.angular_speed
		for i in range(int(rotation_time * self.rate)):
			start_time = timeit.default_timer()
			alpha = i / (rotation_time * self.rate)
			# calculate intermediate angle
			x = cur_x + alpha * dx
			y = cur_y + alpha * dy
			z = cur_z + alpha * dz
			x, y, z = self.normalize(x), self.normalize(y), self.normalize(z)
			interm_euler_angles = (x, y, z)
			# set and publish state with this angle
			quaternion = tf.transformations.quaternion_from_euler(x, y, z)
			self.state.pose.orientation.x = quaternion[0]
			self.state.pose.orientation.y = quaternion[1]
			self.state.pose.orientation.z = quaternion[2]
			self.state.pose.orientation.w = quaternion[3]
			self.state_publisher.publish(self.state)
			rospy.sleep(1. / self.rate - (timeit.default_timer() - start_time))
		# set target angle after rotation
		quaternion = tf.transformations.quaternion_from_euler(trg_x, trg_y, trg_z)
		self.state.pose.orientation.x = quaternion[0]
		self.state.pose.orientation.y = quaternion[1]
		self.state.pose.orientation.z = quaternion[2]
		self.state.pose.orientation.w = quaternion[3]
		self.state_publisher.publish(self.state)

	def move_forward_to(self, target_x, target_y):
		current_pose = self.state.pose.position
		current_x, current_y = current_pose.x, current_pose.y
		dx, dy = target_x - current_x, target_y - current_y
		# move robot with speed of self.linear_speed
		moving_time = math.sqrt(dx ** 2 + dy ** 2) / self.linear_speed
		for i in range(int(moving_time * self.rate)):
			start_time = timeit.default_timer()
			alpha = i / (moving_time * self.rate)
			# calculate intermediate position
			x = current_x + alpha * dx
			y = current_y + alpha * dy
			interm_pose = Pose()
			interm_pose.orientation = self.state.pose.orientation
			interm_pose.position.x = x
			interm_pose.position.y = y
			interm_pose.position.z = self.ground_z
			# set and publish state with this position
			self.state.pose = interm_pose
			self.state_publisher.publish(self.state)
			rospy.sleep(1. / self.rate - (timeit.default_timer() - start_time))
		# after smooth moving, set target position
		self.state.pose.position.x = target_x
		self.state.pose.position.y = target_y
		self.state_publisher.publish(self.state)

	def move_to(self, x, y):
		# moves robot to point (x, y, 0) in GLOBAL coordinate system
		true_state = self.get_state(self.model_name, '')
		self.state.pose = true_state.pose
		position = self.get_position()
		cur_x = position.x
		cur_y = position.y
		cur_z = position.z
		self.rotate_to(math.atan2(y - cur_y, x - cur_x))
		self.move_forward_to(x, y)