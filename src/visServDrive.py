#!/usr/bin/env python
import blobDetectorForDrive as bd
import rospy
from std_msgs.msg import Int32
from ackermann_msgs.msg import AckermannDriveStamped

class visServDrive:
	def __init__(self):
		rospy.init_node("visServDrive")
		
		self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size = 1)
		self.pos_sub = rospy.Subscriber("/visServDrive/position", Int32, self.callback)
		self.blobDect = bd.Echo()
		
		self.centerPt_x = 640		
		
		self.drive_msg = AckermannDriveStamped()
		self.drive_msg.drive.speed = 0.2
		self.doneFlag = False
		self.dist2cent = 0

		self.pub()

	def callback(self, msg):
		x = msg.data
		print "visServDrive callback working"
		if x == 0:
			self.drive_msg.drive.steering_angle = 0.0
			return		
		elif x == -1337:
			self.drive_msg = AckermannDriveStamped()
			self.doneFlag = True
			rospy.loginfo("Parked")
		error = x - self.centerPt_x
		"""if abs(error) <= 30: # if error is tiny
			print "perf"
			self.drive_msg.drive.steering_angle = 0.0
		#elif error > 30: # if item is right, steer right
			print "go right"
			self.drive_msg.drive.steering_angle = -0.2 * abs(error)
		#else: # if item is left, steer left
			print "go left"
			self.drive_msg.drive.steering_angle = 0.2 * abs(error) """
		self.drive_msg.drive.steering_angle = -0.01 * error

	def pub(self):
		while not rospy.is_shutdown():
			self.drive_pub.publish(self.drive_msg)
			if self.doneFlag == True:
				break
			rospy.Rate(4).sleep()
if __name__ == "__main__":
	v = visServDrive()
	rospy.spin()
		
		
		
		 


