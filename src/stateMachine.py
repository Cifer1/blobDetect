#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String, Int32
from cv_bridge import CvBridge, CvBridgeError
import threading

class Main:
	def __init__(self):
		self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.callback)
		self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
		self.drive_msg = AckermannDriveStamped()
		self.drive_msg.drive.speed = 1.0		
		self.thread_lock = threading.Lock()
		self.state = 0
		self.color = None
		self.Image = None
		self.driveflag = None

		self.stateMachine()
		self.drive()
	
	def drive(self):
		while not rospy.is_shutdown():
			self.drive_pub.publish(self.drive_msg)
			rospy.Rate(4).sleep()

	def callback(self, msg):
		self.Image = msg
		self.stateMachine()

	def stateMachine(self):
		if self.state == 0:
			state0()

		elif self.state == 1:
			state1()

		elif self.state == 2:
			state2()

		elif self.state == 3:
			state3()
		
		elif self.state == 4:
			state4()
		
		elif self.state == 5:
			state5()
	
	def state0(self): # finding color using threading
		thread = threading.Thread(target=state0_sub,args=(self.Image,))
		thread.setDaemon(True)
		thread.start()

	def state0_sub(self, image):
		if not self.thread_lock.acquire(False):
            		return
       		image_cv = self.bridge.imgmsg_to_cv2(image)
       		try:
			print "tried find color"            
			self.color = self.findColor(image_cv)
		except CvBridgeError as e:
		    print(e)
		self.thread_lock.release()
		self.state == 1
		self.stateMachine()
	
	def state1(self): # starting drive to color
		thread = threading.Thread(target=state1_sub, args=(self.Image,))
		thread.setDaemon(True)
		thread.start()

	def state1_sub(self, image):
		if not self.thread_lock.acquire(False):
            		return
       		image_cv = self.bridge.imgmsg_to_cv2(image)
       		try:
			print "tried drive to flag"            
			self.driveflag = self.colorSteer(image_cv)
		except CvBridgeError as e:
		    print(e)
		self.thread_lock.release()

		

			
				
		
	
	

