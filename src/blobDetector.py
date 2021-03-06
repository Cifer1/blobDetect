#!/usr/bin/env python
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import threading

#cv2.boundingRectangle

class Echo:
    def __init__(self):
        self.node_name = "Echo"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color",\
                Image, self.cbImage)
        self.pub_image = rospy.Publisher("~echo_image",\
                Image, queue_size=1)
        self.bridge = CvBridge()


        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbImage(self,image_msg):
	rospy.loginfo("cbImage")        
	thread = threading.Thread(target=self.processImage,args=(image_msg,))
	thread.setDaemon(True)
        thread.start()

    def contour(self, image_cv):
	print "hi"
	hsv = cv2.cvtColor(image_cv, cv2.cv.CV_BGR2HSV)	

	binMask = cv2.inRange(hsv, np.array([0,180,180], np.uint8), np.array([12,255,255],np.uint8))	
	masked = cv2.bitwise_and(hsv,hsv,mask=binMask)
	contours, _ = cv2.findContours(binMask,cv2.cv.CV_RETR_EXTERNAL,cv2.cv.CV_CHAIN_APPROX_SIMPLE)
	for c in contours:
		x,y,w,h = cv2.boundingRect(c)
		print w, h		
		if w*h < 500:
			print "cont'd"
			continue

		cv2.rectangle(image_cv, (x,y),(x+w, y+h),(0,255,0),2)
	#cv2.imshow("final", image_cv)
	#cv2.waitKey(0)
	final = self.bridge.cv2_to_imgmsg(image_cv, encoding="bgr8")
	self.pub_image.publish(final)
	
    def processImage(self, image_msg):
	if not self.thread_lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)
        try:
		print "tried"            
		self.contour(image_cv)
	    
        except CvBridgeError as e:
            print(e)
        self.thread_lock.release()
    
    
	

if __name__=="__main__":
    rospy.init_node('Echo')
    e = Echo()
    rospy.spin()

