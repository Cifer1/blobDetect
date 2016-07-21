#!/usr/bin/env python
from sensor_msgs.msg import Image
from std_msgs.msg import String
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
class decision:
	def __init__(self):
		self.node_name = "DecisionMaker"		
		# init node
		rospy.init_node("decisionMaker")		
		# init pubs and subs	
		self.img_sub = rospy.Subscriber("/camera/rgb/image_rect_color",\
                	Image, self.whatColor)
		self.shutdown_sub = rospy.Subscriber("/shutdown", \
			String, self.shutdown)		
		self.result_pub = rospy.Publisher("/decision", String, queue_size=1)
		self.bridge = CvBridge()
		self.RED_LOWER = np.array([0,0,0]) # change these vals
		self.RED_UPPER = np.array([0,0,0])
		self.GRN_LOWER = np.array([0,0,0])
		self.GRN_UPPER = np.array([0,0,0])
		self.results = ("RED","", "GREEN")
		self.bounds = (self.RED_LOWER, self.RED_UPPER, self.GRN_LOWER, self.GRN_UPPER)

		rospy.loginfo("[%s] initialized", %(self.node_name))
	
	def whatColor(self,image_msg):
		
		rospy.loginfo("whatColor begun")
		image_cv = self.bridge.imgmsg_to_cv2(image_msg)
		for poss in range(4)
			hsv = cv2.cvtColor(image_cv, cv2.cv.CV_BGR2HSV)	

			binMask = cv2.inRange(hsv, self.bounds[poss], self.bounds[poss+1])	
			masked = cv2.bitwise_and(hsv,hsv,mask=binMask)
	
			contours, _ = cv2.findContours(binMask,cv2.cv.CV_RETR_EXTERNAL,cv2.cv.CV_CHAIN_APPROX_SIMPLE)
			for c in contours:
				x,y,w,h = cv2.boundingRect(c)
				if w*h > 500:
					self.result_pub(self.results[poss])
			poss += 2

	def shutdown(self, msg):
		if msg == "decisionMaker shutdown": 
			rospy.loginfo("DecisionMaker shutting down")
			rospy.signal_shutdown("dm finished with ops")
		else:
			return
