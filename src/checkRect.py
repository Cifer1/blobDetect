#!/usr/bin/env python
import numpy as np
import rospy
from std_msgs.msg import Float32, Bool, Int32
from sensor_msgs.msg import Image
import threading
import cv2
from cv_bridge import CvBridge, CvBridgeError

class checkRect:
	def __init__(self):
		# init the node
		rospy.init_node("checkRect")
		# init the pubs and subs
		self.img_sub = rospy.Subscriber("/camera/rgb/image_rect_color",\
               		Image, self.cbImage)
		self.img_pub = rospy.Publisher("/echo",Image, queue_size = 1)
		self.angle_pub = rospy.Publisher("/steer_angle",\
			Float32, queue_size=3)		
		self.done_pub = rospy.Publisher("/rect_done",Bool, queue_size=1)
		self.color_pub = rospy.Publisher("/color", Int32, queue_size=1)
		self.bridge = CvBridge()
		self.delivered = False
		self.flag = True
		self.color = None # 0 is red, 1 is green

		self.RED_LOWER = np.array([0,130,130])
		self.RED_UPPER = np.array([10,255,255])
		self.GRN_LOWER = np.array([60,100,100])
		self.GRN_UPPER = np.array([77,255,255])
		self.final_lower = None
		self.final_upper = None
		self.thread_lock = threading.Lock()
	def cbImage(self,image_msg):
		if not self.delivered:
			self.cbImageH(image_msg)
        def cbImageH(self,image_msg):
		rospy.loginfo("cbImage")        
		thread = threading.Thread(target=self.processImage,args=(image_msg,))
		thread.setDaemon(True)
		thread.start()

   	def contour(self, image_cv):
		hsv = cv2.cvtColor(image_cv, cv2.cv.CV_BGR2HSV)	# convert to hsv
		if self.color == None:
			rospy.loginfo(str(self.flag))
			binMask = cv2.inRange(hsv, self.RED_LOWER, self.RED_UPPER) # take red mask
			binMask2 = cv2.inRange(hsv, self.GRN_LOWER, self.GRN_UPPER) # take green mask 

			contours_red,_  = cv2.findContours(binMask, cv2.cv.CV_RETR_EXTERNAL, cv2.cv.CV_CHAIN_APPROX_SIMPLE) # red contour
			contours_grn,_ = cv2.findContours(binMask2, cv2.cv.CV_RETR_EXTERNAL, cv2.cv.CV_CHAIN_APPROX_SIMPLE) # green contour		
			flag = "Green"
			print "color contours taken"
			bigR = self.pickBiggest(contours_red) if len(contours_red) > 0 else None
			bigG = self.pickBiggest(contours_grn) if len(contours_grn) > 0 else None
			if bigR != None and bigG != None and cv2.contourArea(bigR) > cv2.contourArea(bigG):
				self.color = 0
			elif bigR != None and bigG != None and cv2.contourArea(bigG) > cv2.contourArea(bigR):
				self.color = 1
			elif bigR != None:
				self.color = 0
			elif bigG != None:
				self.color = 1
			self.color = 1
			if self.color == 0: # sets mask bounds for red
				self.final_lower = self.RED_LOWER 
				self.final_upper = self.RED_UPPER
			else: # sets mask bounds for green
				self.final_lower = self.GRN_LOWER
				self.final_upper = self.GRN_UPPER
			self.color_pub.publish(self.color)
			
		print "color " + str(self.color)
		binMask = cv2.inRange(hsv, self.final_lower, self.final_upper) # binary mask
		masked = cv2.bitwise_and(hsv,hsv,mask=binMask) 
	
		contours, _ = cv2.findContours(binMask,cv2.cv.CV_RETR_EXTERNAL,cv2.cv.CV_CHAIN_APPROX_SIMPLE) # contouring
		#print len(contours)
		msg = 0
		c = self.pickBiggest(contours) if len(contours)>0 else None
		x,y,w,h = cv2.boundingRect(c)
		if w*h>20000:
			print "big"
			self.flag = False

		#for c in contours:
			#print "contour loop"
			#x,y,w,h = cv2.boundingRect(c) # find the rect around the contour
			#if w*h < 2000: # if too small, pay no attention
				#print "cont'd"
				#continue
			#elif h > 300: # if too big, end the node
				#print "big"
				#self.flag = False
				#continue
		msg = x + 0.5 * w # finds x val of center of contour 
			#cv2.rectangle(image_cv, (x,y),(x+w, y+h),(0,255,0),2)
		print str(msg)
		if self.flag == False and self.delivered == False: # if we're done
			self.done_pub.publish(self.flag) # say we're done
			rospy.loginfo(str(self.flag))
			self.delivered = True
			
		error = msg - 640 # distance from center
		if abs(error) != 640:
			self.angle_pub.publish(error * -0.001) # publishes P controlled angle for driving 

		self.img_pub.publish(self.bridge.cv2_to_imgmsg(image_cv))
		

		#cv2.imshow("masked",image_cv)
		#cv2.waitKey(0)
		#cv2.destroyAllWindows()
	
	def pickBiggest(self,contours):
		result = contours[0]
		for x in contours:
			if cv2.contourArea(x)>cv2.contourArea(result):
				result = x
		return result
        def processImage(self, image_msg):
		print "processIm"
		if not self.thread_lock.acquire(False):
		    #print("failed")
		    return
		image_cv = self.bridge.imgmsg_to_cv2(image_msg)
		#print "about to try"
		try:
			print "tried"            
			self.contour(image_cv)
		    
		except CvBridgeError as e:
		    print(e)
		self.thread_lock.release()
if __name__ == "__main__":
	rospy.init_node("checkRect")
	e = checkRect()
	rospy.spin()

		
		
