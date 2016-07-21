#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Int32, Bool, Float32
#from sensor_msgs.msg import
from blobDetectorForDrive import Echo
# step1: first init the main node
# step2: detect color and start colordrive
# step3: once colorDrive is done, stop node and start turn
# step4: once turn is complete, start wall follow
class mainNode:
	def __init__(self):
	
		#init pubs and subs
		self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)
		self.color_sub = rospy.Subscriber("/color", Int32, self.colorCB)
		self.rectdone_sub = rospy.Subscriber("/rect_done", Bool, self.step3)
		self.angle_sub = rospy.Subscriber("/steer_angle", Float32, self.driveUpdate)
		self.start_pub = rospy.Publisher("/follow_start", Bool, queue_size = 1)
		self.side_pub = rospy.Publisher("/side", Int32, queue_size=1)
		self.drive_msg = AckermannDriveStamped()
		self.drive_msg.drive.speed = 1.0		
		
		self.color = None
		self.drive()

	def drive(self):
		while not rospy.is_shutdown():			
			self.drive_pub.publish(self.drive_msg)
			rospy.Rate(4).sleep()

	def driveUpdate(self, msg):
		self.drive_msg.drive.steering_angle = msg

	def colorCB(self, msg):
		self.color = msg
		if self.color == 0: print "red"
		else: print "green"
		

	def step3(self,msg):
		self.drive_msg.drive.speed = 0.0
		time = rospy.Time.now()
		while rospy.Time.now() - time < 1:
			self.drive_msg.drive.speed = -1.0
		self.drive_msg.drive.speed = 0.0			
		if self.color == 0:
			self.drive_msg.drive.steering_angle = 1.0
		else:
			self.drive_msg.drive.steering_angle = -1.0
		time = rospy.Time.now()		
		while rospy.Time.now() - time < 1:
			self.drive_msg.drive.speed = 1.0
		self.drive_msg.drive.speed = 0.0
		self.step4()
	
	def step4(self):
		if self.color == 0:
			self.side_pub.publish(1)
		else:
			self.side_pub.publish(2)		
		self.start_pub.publish(True)
		self.drive_msg.drive.speed = 1.0
		
		
					

			
		
		
	
if __name__ == "__main__":
	rospy.init_node("mainNode")
	f = mainNode()
	rospy.spin()
