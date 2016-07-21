#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32, Bool, Int32
from sensor_msgs.msg import Image

class checkRect:
	def __init__(self):
		# init the node
		rospy.init_node("checkRect")
		# init the pubs and subs
		self.img_sub = rospy.Subscriber("/camera/rgb/image_rect_color",\
               		Image, self.callback)
		self.error_pub = rospy.Publisher("/error",\
			Float32, queue_size=3)		
		self.done_pub = rospy.Publisher("/rect_done",Bool, queue_size=1)
		self.color_pub = rospy.Publisher("/color", Int32, queue_size=1)

		self.flag = True
		self.color = None # 0 is red, 1 is green

		
		
