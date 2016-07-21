#!/usr/bin/env python
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan

sub = rospy.Subscriber("scan", LaserScan, safety_mech) 
rospy.init_node('safety')
rate = rospy.Rate(10)
pub = rospy.Publisher("vesc/ackermann_cmd_mux/input/safety", AckermannDriveStamped, queue_size=10)

publish = False
def safety_mech(msg):
	ranges = msg.ranges
	for i in range(450, 631+1):
		if((ranges[i] < 0.51) &(ranges[i+1] < 0.51)):
			pub.publish(AckermannDriveStamped())
			print "stop" + str(ranges[i])

rospy.spin()
