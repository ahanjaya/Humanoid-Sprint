#! /usr/bin/env python3
import math
import rospy
import numpy as np

from time import sleep
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32
from robot_msgs.msg import HeadMove, Button, ArucoData 

class Player:
	def __init__(self):
		rospy.init_node('sprint_player')
		rospy.loginfo("[Player] Sprint Player - Running")

		self.freq		= 60.0
		self.main_rate	= rospy.Rate(self.freq)
		self.frame_w    = rospy.get_param("/usb_cam/image_width")
		self.frame_h    = rospy.get_param("/usb_cam/image_height")
		# self.frame_w      = rospy.get_param("/uvc_webcam/width")
		# self.frame_h      = rospy.get_param("/uvc_webcam/height")
		
		self.init_state = True
		self.state 		= 'initial'
		self.debug		= False
		self.rocknroll  = True
		self.marker		= { 'x': -1, 'y': -1, 'size': -1 }

		# button
		self.middle_button = { 'last': False, 'current': False }
		self.right_button  = { 'last': False, 'current': False }

		# head
		self.head = { 'pan': 0.0,  'tilt': 0.0}
		self.pan  = { 'min': -1.0, 'max':  1.0}
		self.tilt = { 'min': -1.6, 'max': -0.5}
		self.pos_tilt  = -0.8
		self.pan_step  = 0 
		self.tilt_step = 0 
		self.cnt_loss  = 0
		

		self.sum_err_pan  = 0
		self.sum_err_tilt = 0
		self.last_error_x = 0
		self.last_error_y = 0

		# walk
		self.bwd_xcomp = True

		# Subscriber
		rospy.Subscriber("/sprint/marker/position", ArucoData,	self.marker_pos_callback)
		rospy.Subscriber("/button/state", 			Button,     self.button_callback)

        # Publisher
		self.walk_pub    = rospy.Publisher("/walk/velocity", Twist,    queue_size=1)
		self.xcomp_pub   = rospy.Publisher("/walk/footcomp", Float32,  queue_size=1)	
		self.motion_pub	 = rospy.Publisher("/motion/state",  String,   queue_size=1)
		self.head_pub	 = rospy.Publisher("/head/pos",      HeadMove, queue_size=1)
		self.gripper_pub = rospy.Publisher("/gripper/state", Bool,     queue_size=1)	

	def marker_pos_callback(self, msg):
		self.marker['x']    = msg.x
		self.marker['y']    = msg.y
		self.marker['size'] = msg.size

		if self.debug:
			rospy.loginfo('[Player] Marker: {}'.format(self.marker))

	def button_callback(self, msg):
		self.middle_button['current'] = msg.middle
		self.right_button['current']  = msg.right

		# detect button transition
		# middle button
		if self.middle_button['last'] != self.middle_button['current']:
			if self.middle_button['current']:
				if self.rocknroll:
					rospy.loginfo("[Player] Sit down")
					self.motion_pub.publish("sit")
					self.state     = None
					self.rocknroll = False
				else:
					rospy.loginfo("[Player] Standup")
					self.motion_pub.publish("stand")
					self.state	   = 'initial'
					self.rocknroll = True
		self.middle_button['last'] = self.middle_button['current']

		# right button
		if self.right_button['last'] != self.right_button['current']:
			if self.right_button['current']:
				pass
		self.right_button['last'] = self.right_button['current']

		# print(self.button['middle'], self.button['right'])
		if self.debug:
			rospy.loginfo('[Player] Button: {}'.format(self.button))

	def head_limit(self, pos_pan, pos_tilt):
		# pan limiter
		if pos_pan <= self.pan['min'] :		
			pos_pan = self.pan['min']
		elif pos_pan >= self.pan['max'] :		
			pos_pan = self.pan['max']

		# tilt limiter
		if pos_tilt <= self.tilt['min'] :
			pos_tilt = self.tilt['min']
		elif pos_tilt >= self.tilt['max'] :
			pos_tilt = self.tilt['max']
		
		head_pos      = HeadMove()
		head_pos.pan  = self.head['pan']  = round(pos_pan,  4)
		head_pos.tilt = self.head['tilt'] = round(pos_tilt, 4)
		return head_pos

	def head_move(self, pos_pan, pos_tilt):
		head_pos      = HeadMove()
		head_pos.pan  = self.head['pan']  = round(pos_pan,  4)
		head_pos.tilt = self.head['tilt'] = round(pos_tilt, 4)
		self.head_pub.publish(head_pos)

	def walk(self, x, y, a):
		velocity = Twist()
		velocity.linear.x = x
		velocity.linear.y = y
		velocity.linear.z = a
		self.walk_pub.publish(velocity)

	def marker_lost(self, threshold):
		if self.marker['x'] == -1 and self.marker['y'] == -1:		
			self.cnt_loss += 1
			if self.cnt_loss >= threshold :
				return True
		else :
			self.cnt_loss = 0
			return False

	def scan_marker(self, mode):
		if mode == "only_tilt":
			self.head['pan']  =  0.0
			self.pos_tilt += self.tilt_step
			# print('tilt ', self.pos_tilt, self.tilt_step)
			if self.pos_tilt >= -0.8 or self.pos_tilt <= self.tilt['min']:
				self.tilt_step *= -1

		head_pos = self.head_limit(self.head['pan'], self.pos_tilt)
		self.head_pub.publish(head_pos)
		# print(head_pos)

	def track_marker(self):
		dt = 1.0 / self.freq
		Kp_pan  = rospy.get_param("/sprint_params/player/Pan_Kp")
		Ki_pan  = rospy.get_param("/sprint_params/player/Pan_Ki")
		Kd_pan  = rospy.get_param("/sprint_params/player/Pan_Kd")
		Kp_tilt = rospy.get_param("/sprint_params/player/Tilt_Kp")
		Ki_tilt = rospy.get_param("/sprint_params/player/Tilt_Ki")
		Kd_tilt = rospy.get_param("/sprint_params/player/Tilt_Kd")

		if self.marker['x'] != -1 and self.marker['y'] != -1:
			# pan
			error_x = (self.frame_w/2) - self.marker['x']
			error_x *= 77.32 / self.frame_w
			error_x = (error_x * math.pi)/ 180
			error_x_diff = error_x - self.last_error_x

			P_pan              = self.last_error_x * Kp_pan
			self.sum_err_pan  += error_x * dt
			I_pan              = self.sum_err_pan * Ki_pan
			deriv_err_pan      = error_x_diff / dt
			D_pan              = deriv_err_pan * Kd_pan
			self.last_error_x  = error_x
			self.head['pan']  += (P_pan + I_pan + D_pan)

			# tilt
			error_y = (self.frame_h/2) - self.marker['y']
			error_y *= -1
			error_y *= 61.93 / self.frame_h
			error_y = (error_y * math.pi) /180
			error_y_diff = error_y - self.last_error_y

			P_tilt            = self.last_error_y * Kp_tilt
			self.sum_err_tilt += error_y * dt
			I_tilt            = self.sum_err_tilt * Ki_tilt
			deriv_err_tilt    = error_y_diff / dt
			D_tilt            = deriv_err_tilt * Kd_tilt
			self.last_error_y = error_y
			self.head['tilt'] += (P_tilt + I_tilt + D_tilt)

		head_pos = self.head_limit(self.head['pan'], self.head['tilt'])
		self.head_pub.publish(head_pos)

	def body_track_marker(self, mode):
		Kp_fwd = rospy.get_param("/sprint_params/player/Body_Fwd_Kp")
		Kp_bwd = rospy.get_param("/sprint_params/player/Body_Bwd_Kp")
		
		error_body_a = self.head['pan'] - 0
		max_walk_a   = 0.4
		
		if mode == 'forward':
			body_a = error_body_a * Kp_fwd
		elif mode == 'backward':
			body_a = error_body_a * Kp_bwd

		if body_a >= max_walk_a:
			body_a = max_walk_a
		elif body_a <= -max_walk_a:
			body_a = -max_walk_a
		
		return round(body_a, 4)

	def run(self):
		sleep(2.5)

		while not rospy.is_shutdown():
			if self.init_state:
				self.tilt_step = rospy.get_param("/sprint_params/player/Tilt_Step") * -1
				self.pan_step  = rospy.get_param("/sprint_params/player/Pan_Step")
				
				rospy.loginfo("[Player] First Robot Standup")
				self.motion_pub.publish("stand")
				self.init_state = False
				self.rocknroll  = True

			##########################
			###### Rock n Roll #######
			##########################
			if self.rocknroll:
				if self.state == 'initial':
					# rospy.loginfo("[Player] State: {}".format(self.state))

					if self.marker_lost(20):
						# head_move(0.0, -1.3)
						self.scan_marker('only_tilt')
					else:
						self.track_marker()
						if self.right_button['current']:
							self.state = 'forward'
							# self.state = 'backward'
							rospy.loginfo("[Player] State: {}".format(self.state))
					# self.state = None

				elif self.state == 'warmup':
					rospy.loginfo("[Player] State: {}".format(self.state))
					self.state = None

				elif self.state == 'forward':
					# rospy.loginfo("[Player] State: {}".format(self.state))
					
					if self.marker_lost(20):
						self.scan_marker('only_tilt')
						self.walk(0.06, 0.0, 0.00)
					else:
						self.track_marker()
						if self.marker['size'] < 60000:
							rotate = self.body_track_marker('forward')
							self.walk(0.06, 0.0, rotate)
						else:
							self.state = "backward"
							rospy.loginfo("[Player] State: {}".format(self.state))

				elif self.state == 'backward':
					# rospy.loginfo("[Player] State: {}".format(self.state))
					if self.bwd_xcomp: 
						bwd_xcomp = -0.005
						self.xcomp_pub.publish(bwd_xcomp)
						rospy.loginfo("[Player] Backward foot xcomp: {}".format(bwd_xcomp))
						self.bwd_xcomp = False

					if self.marker_lost(20):
						self.scan_marker('only_tilt')
						self.walk(-0.05, 0.0, 0.00)
					else:
						self.track_marker()
						rotate = self.body_track_marker('backward')
						self.walk(-0.05, 0.0, rotate)

				else:
					if self.state is not None:
						rospy.loginfo("[Player] State: {}".format(self.state))

			self.main_rate.sleep()

if __name__ == '__main__':
    player = Player()
    player.run()