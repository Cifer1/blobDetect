#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import 
from sensor_msgs.msg import 
from decisionMaker import decision
from blobDetectorForDrive import Echo
# step1: first init the main node
# step2: second decision making node
# step3: third start color follow node with params of color decided
# step4: fourth turn left/right OR use sep node to do that 
# step5: fifth wall follow

class mainNode:
	def __init__(self):
		rospy.init_node("mainNode") #step1 complete
	
		#init pubs and subs
		self.shutdown_pub = rospy.Publisher("/shutdown", String, queue_size = 10)
		self.decision_sub = rospy.Subscriber("/decision", String, self.step3)
		self.colordone_sub = rospy.Subscriber("/color_done", String, self.step4)
		
		decision() #step2 initialized

	def step3(self,msg):
		self.shutdown_pub.publish("decisionMaker shutdown") # decision shutdown, step2 really complete		
		if msg == "RED":
			Echo("red")
		else:
			Echo("green")
		
	
	
