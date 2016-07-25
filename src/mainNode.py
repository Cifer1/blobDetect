#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan
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
		self.angle1_sub = rospy.Subscriber("/steer_angle", Float32, self.driveUpdate1)
		self.angle2_sub = rospy.Subscriber("/steer_angle_follow", Float32, self.driveUpdate2)
		self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan)
		self.side_pub = rospy.Publisher("/side", Int32, queue_size=1)
		self.drive_msg = AckermannDriveStamped()
		self.drive_msg.drive.speed = 1.0
		self.scanVal = 0	
		self.back = False	
		self.angle_flag = 1
		self.runStep = True
		self.color = None
		self.drive()

	def drive(self):
		while not rospy.is_shutdown():			
			self.drive_pub.publish(self.drive_msg)
			print(self.drive_msg.drive.steering_angle)
			print(self.drive_msg.drive.speed)
			rospy.Rate(8).sleep()
	def scan(self, msg):
		self.scanVal = msg.ranges[540]
		if self.back and self.scanVal < 1.5:					
			self.drive_msg.drive.steering_angle = 0.0
			self.drive_msg.drive.speed = -0.5
		elif self.back:
			self.back = False
			self.step3H()
			
	def driveUpdate1(self, msg):
		if self.angle_flag == 1:
			self.drive_msg.drive.steering_angle = msg.data
	def driveUpdate2(self, msg):
		if self.angle_flag == 2:
			self.drive_msg.drive.steering_angle = msg.data
	def colorCB(self, msg):
		self.color = msg.data
		if self.color == 0: print "red"
		else: print "green"
		
	def step3(self,msg):
		if self.runStep == True:
			#self.step3H()
			self.back = True
	def step3H(self):
			print("in step 3H")

			if self.color == 0:
				self.drive_msg.drive.steering_angle = -1.0
			else:
				self.drive_msg.drive.steering_angle = -1.0
			time = rospy.Time.now()		
			while rospy.Time.now() - time < rospy.Duration(1.5):
				self.drive_msg.drive.speed = 1.0
			self.drive_msg.drive.speed = 0.0
			self.drive_msg.drive.steering_angle = 0.0
			self.runStep = False
			
			self.step4()
	
	def step4(self):
		print("in step 4")
		if self.color == 0:
			self.side_pub.publish(1)
		else:
			self.side_pub.publish(2)
		self.angle_flag = 2
		self.drive_msg.drive.speed = 1.0
		
		
					

			
		
		
	
if __name__ == "__main__":
	rospy.init_node("mainNode")
	f = mainNode()
	rospy.spin()
