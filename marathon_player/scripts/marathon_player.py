#! /usr/bin/env python3
import math
import rospy
import numpy as np

from time import sleep, time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String, Bool, Float32
from robot_msgs.msg import HeadMove, Button 

class Player:
	def __init__(self):
		rospy.init_node('marathon_player')
		rospy.loginfo("[Player] Marathon Player - Running")

		self.freq		= 60.0
		self.main_rate	= rospy.Rate(self.freq)
		self.frame_w    = rospy.get_param("/usb_cam/image_width")
		self.frame_h    = rospy.get_param("/usb_cam/image_height")

		# button
		self.middle_button = { 'last': False, 'current': False }
		self.right_button  = { 'last': False, 'current': False }

		self.init_variable()

		# Subscriber
		rospy.Subscriber("/button/state",           Button, self.button_callback)
		rospy.Subscriber("/marathon/line/position", Pose2D,	self.line_pos_callback)
		rospy.Subscriber("/marathon/marker/arrow",  String,	self.arrow_pos_callback)

        # Publisher
		self.walk_pub    = rospy.Publisher("/walk/velocity", Twist,    queue_size=1)
		self.motion_pub	 = rospy.Publisher("/motion/state",  String,   queue_size=1)
		self.head_pub	 = rospy.Publisher("/head/pos",      HeadMove, queue_size=1)

	def init_variable(self):
		self.init_state = True
		self.state 		= 'initial'
		self.debug		= False
		self.rocknroll  = True
		self.arrow      = None
		self.line		= { 'x': -1, 'y': -1, 'size': -1 }
		# self.rleft_th,  self.rright_th  = 1.8,  1.8 # sec
		self.rleft_th,  self.rright_th  = 200,  200 # counter
		self.vel_x,     self.vel_a      = 0.03, 0.3
		self.line_conf, self.line_th    = 0,    150 # counter

		# head
		self.head = { 'pan': 0.0,  'tilt': 0.0}
		self.pan  = { 'min': -1.0, 'max':  1.0}
		self.tilt = { 'min': -1.6, 'max': -0.5}
		self.fixed_tilt  = -0.5
		self.pos_tilt,     self.pos_pan      = -0.8, 0.0
		self.pan_step,     self.tilt_step    = 0, 0
		self.sum_err_pan,  self.sum_err_tilt = 0, 0
		self.last_error_x, self.last_error_y = 0, 0
		self.cnt_loss    = 0
		self.scan_marker = False

		self.count_left  = 0

	def arrow_pos_callback(self, msg):
		self.arrow = msg.data

		# if self.state == 'follow_line':
		if self.scan_marker:
			if self.arrow == "straight":
				self.state = 'straight'

			elif self.arrow == "left":
				self.state = 'rotate_left'

			elif self.arrow == "right":
				self.state = 'rotate_right'

	def button_callback(self, msg):
		self.middle_button['current'] = msg.middle
		self.right_button['current']  = msg.right

		# button transition
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

	def line_pos_callback(self, msg):
		self.line['x'] = msg.x
		self.line['y'] = msg.y

		if self.debug:
			rospy.loginfo('[Player] Line: {}'.format(self.marker))

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

	def line_lost(self, threshold):
		if self.line['x'] == -1 and self.line['y'] == -1:		
			self.cnt_loss += 1
			if self.cnt_loss >= threshold :
				return True
		else :
			self.cnt_loss = 0
			return False

	def scan_line(self, mode):
		if self.pan_step > 0:        
			self.pan_step = rospy.get_param("/marathon_params/player/Pan_Step")
		else:               
			self.pan_step = rospy.get_param("/marathon_params/player/Pan_Step") * -1

		if mode == "only_tilt":
			# print('scan tilt')
			self.pos_pan  =  0.0
			self.pos_tilt += self.tilt_step
			if self.pos_tilt >= -0.5 or self.pos_tilt <= self.tilt['min']:
				self.tilt_step *= -1

		elif mode == "only_pan":
			self.pos_tilt  =  self.fixed_tilt
			self.pos_pan  += self.pan_step
			if self.pos_pan >= self.pan['max'] or self.pos_pan <= self.pan['min']:
				self.pan_step *= -1

		head_pos = self.head_limit(self.pos_pan, self.pos_tilt)
		self.head_pub.publish(head_pos)
		# print(head_pos)

	def track_line(self, tilt=False):
		dt = 1.0 / self.freq
		Kp_pan  = rospy.get_param("/marathon_params/player/Pan_Kp")
		Ki_pan  = rospy.get_param("/marathon_params/player/Pan_Ki")
		Kd_pan  = rospy.get_param("/marathon_params/player/Pan_Kd")
		Kp_tilt = rospy.get_param("/marathon_params/player/Tilt_Kp")
		Ki_tilt = rospy.get_param("/marathon_params/player/Tilt_Ki")
		Kd_tilt = rospy.get_param("/marathon_params/player/Tilt_Kd")

		if self.line['x'] != -1 and self.line['y'] != -1:
			# pan
			error_x = (self.frame_w/2) - self.line['x']
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

			if tilt:
				error_y = (self.frame_h/2) - self.line['y']
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
			else:
				self.head['tilt'] = rospy.get_param("/marathon_params/player/Tilt_Angle")

		head_pos = self.head_limit(self.head['pan'], self.head['tilt'])
		self.head_pub.publish(head_pos)

	def body_track_line(self):
		Kp_fwd = rospy.get_param("/marathon_params/player/Body_Kp")
		
		error_body_a = self.head['pan'] - 0
		max_walk_a   = 0.4
		body_a = error_body_a * Kp_fwd

		if body_a >= max_walk_a:
			body_a = max_walk_a
		elif body_a <= -max_walk_a:
			body_a = -max_walk_a
		
		return round(body_a, 4)

	def run(self):
		sleep(2.5)

		while not rospy.is_shutdown():
			if self.init_state:
				self.tilt_step = rospy.get_param("/marathon_params/player/Tilt_Step") * -1
				self.pan_step  = rospy.get_param("/marathon_params/player/Pan_Step")
				
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
					# self.head_move(0.0, self.fixed_tilt)

					if self.line_lost(20):
						self.scan_line('only_pan')
					else:
						self.track_line()
						if self.right_button['current']:
							self.state = 'follow_line'
							rospy.loginfo("[Player] State: {}".format(self.state))

				elif self.state == 'follow_line':
					# rospy.loginfo("[Player] State: {}".format(self.state))
					if self.line_lost(10):
						# self.scan_line('only_pan')
						# self.walk(0.0, 0.0, 0.0)
						self.scan_marker = True
						self.motion_pub.publish("stop")
					else:
						self.track_line()
						rotate = self.body_track_line()
						self.walk(self.vel_x, 0.0, rotate)

				elif self.state == 'straight':
					# rospy.loginfo("[Player] State: {}".format(self.state))
					self.scan_marker = False
					self.walk(self.vel_x, 0.0, 0.0)
					
					if self.line_lost(10):
						self.head_move(0.0, self.fixed_tilt-0.3)
						# self.scan_line('only_tilt')
					else:
						self.track_line(tilt=True)
						self.line_conf += 1

					if self.line['x'] != -1 and self.line['y'] != -1:
						if self.line_conf >= self.line_th:
							self.state = 'follow_line'
							self.line_conf = 0
					
				elif self.state == 'rotate_left':
					# rospy.loginfo("[Player] State: {}".format(self.state))
					self.scan_marker = False
					
					# start = time()
					# while time()-start <= self.rleft_th:
					# 	# print('rotate left: {}'.format(time()-start))
					
					if self.count_left > self.rleft_th:
						self.state = 'straight'
						self.count_left = 0
					else:
						self.count_left += 1
						self.head_move(0.0, self.fixed_tilt)	
						self.walk(0.0, 0.0, self.vel_a)

				elif self.state == 'rotate_right':
					rospy.loginfo("[Player] State: {}".format(self.state))
					self.scan_marker = False
					
					# start = time()
					# while time()-start <= self.rright_th: 
					# 	# print('rotate right: {}'.format(time()-start))
					# 	self.head_move(0.0, self.fixed_tilt)
					# 	self.walk(0.0, 0.0, -self.vel_a)
					self.head_move(0.0, self.fixed_tilt)
					self.walk(0.0, 0.0, -self.vel_a)
					self.state = 'straight'

				else:
					if self.state is not None:
						rospy.loginfo("[Player] State: {}".format(self.state))

			self.main_rate.sleep()

if __name__ == '__main__':
    player = Player()
    player.run()