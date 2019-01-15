#!/usr/bin/env python

import rospy

from std_msgs.msg import *

from bombel_msgs.msg import BombelSpeed
from bombel_msgs.msg import BombelState
import tf
from math import *

BOMBEL_COM_RATE = 10.0
NODE_NAME = "TEST_PATH"
NODE_NAME_HEADER = "[" + NODE_NAME + "]"

bombelSpeedMsg = BombelSpeed()
bombelStateMsg = BombelState()

speed=100;

if __name__ == '__main__':
	rospy.init_node(NODE_NAME, anonymous=True)

	pub = rospy.Publisher('/bombel/speed', BombelSpeed, queue_size=10)
	
	rate = rospy.Rate(BOMBEL_COM_RATE) # 10hz

	bombelSpeedMsg.joint0_speed = 100
	bombelSpeedMsg.joint1_speed = 0
	bombelSpeedMsg.joint2_speed = 0

	pub.publish(bombelSpeedMsg)

	bombelSpeedMsg.joint0_speed = 0
	bombelSpeedMsg.joint1_speed = 0
	bombelSpeedMsg.joint2_speed = 0

	rospy.loginfo(NODE_NAME_HEADER + "Publishing speed " + str(speed))

	i=0
	while not rospy.is_shutdown():
		# pub.publish(bombelStateMsg)
		i+=1
		if(i == BOMBEL_COM_RATE):
			pub.publish(bombelSpeedMsg)
			rospy.loginfo(NODE_NAME_HEADER + "Closing node ")
			rospy.signal_shutdown("Speed published for 1 sec")
		rate.sleep()

