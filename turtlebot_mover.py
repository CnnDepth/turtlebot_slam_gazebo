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
		         max_angular_speed=0.5,
		         max_linear_speed=0.5,
		         trajectory_type='line',
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
		self.trajectory_type = trajectory_type
		self.eps = 1e-4
		rospy.init_node('move_robot')
		self.state_publisher = rospy.Publisher("/gazebo/set_model_state", ModelState, latch=True)

	def get_position(self):
		return self.state.pose.position

	def get_orientation_angles(self):
		orientation = self.state.pose.orientation
		current_euler_angles = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
		cur_x, cur_y, cur_z = current_euler_angles
		return cur_x, cur_y, cur_z

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
		cur_x, cur_y, cur_z = self.get_orientation_angles()
		current_euler_angles = (cur_x, cur_y, cur_z)
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

	def move_along_circle(self, center_x, center_y, r, src_angle, trg_angle, direction):
		angular_distance = (trg_angle - src_angle) * direction
		if angular_distance < 0:
			angular_distance += 2 * math.pi
		moving_time = angular_distance / self.angular_speed
		for i in range(int(moving_time * self.rate)):
			start_time = timeit.default_timer()
			alpha = i / (moving_time * self.rate)
			angle = self.normalize(src_angle + alpha * angular_distance * direction)
			x = center_x + math.cos(angle) * r
			y = center_y + math.sin(angle) * r
			interm_pose = Pose()
			interm_pose.position.x = x
			interm_pose.position.y = y
			interm_pose.position.z = self.ground_z
			orientation_angle = angle + math.pi / 2. * direction
			quaternion = tf.transformations.quaternion_from_euler(0, 0, orientation_angle)
			interm_pose.orientation.x = quaternion[0]
			interm_pose.orientation.y = quaternion[1]
			interm_pose.orientation.z = quaternion[2]
			interm_pose.orientation.w = quaternion[3]
			self.state.pose = interm_pose
			self.state_publisher.publish(self.state)
			rospy.sleep(1. / self.rate - (timeit.default_timer() - start_time))

	def move_to(self, x, y):
		# moves robot to point (x, y, 0) in GLOBAL coordinate system
		true_state = self.get_state(self.model_name, '')
		self.state.pose = true_state.pose
		position = self.get_position()
		cur_x = position.x
		cur_y = position.y
		cur_z = position.z
		if self.trajectory_type == 'line':
			self.rotate_to(math.atan2(y - cur_y, x - cur_x))
			self.move_forward_to(x, y)
		elif self.trajectory_type == 'circle':
			# calculate r_opt - circle radius which let robot move with maximum linear and angular speed
			r_opt = self.linear_speed / self.angular_speed
			# calculate r_max - maximal circle radius (radius of the circle which connects current location and destination)
			x_angle, y_angle, z_angle = self.get_orientation_angles()
			current_angle = z_angle
			target_angle = math.atan2(y - cur_y, x - cur_x)
			alpha = self.normalize(target_angle - z_angle)
			l = math.sqrt((x - cur_x) ** 2 + (y - cur_y) ** 2)
			r_max = l / (2 * math.sin(min(abs(alpha), math.pi - abs(alpha))))
			# choose r - minimum of r_opt and r_max and find center of a circle with radius r
			r = min(r_opt, r_max)
			if abs(r) < self.eps:
				return
			perp_angle = self.normalize(current_angle + math.pi / 2.)
			if alpha < 0:
				perp_angle = self.normalize(perp_angle + math.pi)
			perp_x = math.cos(perp_angle)
			perp_y = math.sin(perp_angle)
			center_x = cur_x + perp_x * r
			center_y = cur_y + perp_y * r
			# calculate tangent to chosen circle from destination point
			src_circ_angle = self.normalize(perp_angle + math.pi)
			center_to_dst = math.sqrt((center_x - x) ** 2 + (center_y - y) ** 2)
			beta = math.asin(min(r / center_to_dst, 1.))
			if alpha > 0:
				trg_circ_angle = self.normalize(math.atan2(y - center_y, x - center_x) + beta - math.pi / 2.)
			else:
				trg_circ_angle = self.normalize(math.atan2(y - center_y, x - center_x) - beta + math.pi / 2.)
			# move robot along chosen circle, from source angle to target angle, and after move it forwardly to destination
			self.move_along_circle(center_x, center_y, r, src_circ_angle, trg_circ_angle, 2 * (alpha > 0) - 1)
			self.move_forward_to(x, y)
		else:
			raise NotImplementedError