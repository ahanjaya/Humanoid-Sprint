#! /usr/bin/env python3
import os
import rospy
import rospkg
import socket

from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32
from robot_msgs.msg import HeadMove, Button
from std_srvs.srv import Trigger, TriggerResponse

class Bridge:
	def __init__(self):
		rospy.init_node('motion_bridge')
		rospy.loginfo("[Bridge] Motion Bridge - Running")
		rospack          = rospkg.RosPack()
		self.kinematics  = rospack.get_path("kinematics") + "/upenn/Player"
		self.main_rate   = rospy.Rate(60)

		# socket communication
		self.localhost   = "localhost" # "127.0.0.1"
		self.motion_port = 8080
		self.head_port   = 8081
		self.comp_port   = 8082
		self.button_port = 8083

		self.sock        = None
		self.sock_button = None

		self.kill_screen   = "killall screen;"
		self.run_kinematic =  " cd {}; \
								screen -S dcm     lua run_dcm.lua; \
							    screen -S player  lua walk_server.lua;".format(self.kinematics)

		self.gripper_state = False
		self.button_msg    = Button()

		# Subscriber
		rospy.Subscriber("/head/pos",       HeadMove, self.head_callback)
		rospy.Subscriber("/motion/state",   String,   self.motion_state_callback)
		rospy.Subscriber("/walk/velocity",  Twist,    self.velocity_callback)
		rospy.Subscriber("/walk/footcomp",  Float32,  self.foot_comp_callback)
		rospy.Subscriber("/gripper/state",  Bool,     self.gripper_state_callback)

		# Publisher
		self.button_pub = rospy.Publisher("/button/state", Button, queue_size=1)
	
	def head_callback(self, msg):
		MESSAGE = "head {} {}".format(msg.pan, msg.tilt)
		MESSAGE = MESSAGE.encode() # python3
		self.sock.sendto(MESSAGE, (self.localhost, self.head_port))

	def motion_state_callback(self, msg):
		MESSAGE = str.encode(msg.data) # python3
		self.sock.sendto(MESSAGE, (self.localhost, self.motion_port))

	def velocity_callback(self, msg):
		MESSAGE = "walk {} {} {}".format(msg.linear.x, msg.linear.y, msg.linear.z)
		MESSAGE = MESSAGE.encode() # python3
		self.sock.sendto(MESSAGE, (self.localhost, self.motion_port))

	def foot_comp_callback(self, msg):
		MESSAGE = "comp {} ".format(round(msg.data,4))
		MESSAGE = MESSAGE.encode() # python3
		self.sock.sendto(MESSAGE, (self.localhost, self.comp_port))
		print(MESSAGE)

	def gripper_state_callback(self, msg):
		self.gripper_state = msg.data

		if self.gripper_state:
			MESSAGE = "grip 1"
		else:
			MESSAGE = "grip 0"

		MESSAGE = str.encode(msg.data) # python3
		self.sock.sendto(MESSAGE, (self.localhost, self.motion_port))

	def button_process(self):
		raw_data = self.sock_button.recv(1024)
		raw_data = raw_data.decode() # python3
		buttons  = raw_data.split(" ")
		self.button_msg.middle = int(buttons[0])
		self.button_msg.right  = int(buttons[1])
		self.button_pub.publish(self.button_msg)

	def trigger_response(self, request):
		return TriggerResponse(success=True, message="controller run")

	def kill_node(self):
		self.sock.close()
		self.sock_button.close()
		rospy.signal_shutdown("[Bridge] Shutdown Time...") 

	def run (self):
		os.system(self.kill_screen)
		os.system(self.run_kinematic)

		self.sock        = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock_button = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
		self.sock_button.bind((self.localhost, self.button_port))
	
		# rospy.Service('/srv_controller', Trigger, self.trigger_response)
		
		while not rospy.is_shutdown():
			self.button_process()
			# self.main_rate.sleep() # add sleep caused latency of reading button
		
		rospy.on_shutdown(self.kill_node)

if __name__ == '__main__':
    bridge = Bridge()
    bridge.run()