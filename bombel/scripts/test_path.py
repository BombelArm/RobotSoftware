#!/usr/bin/env python

import rospy

from std_msgs.msg import *

from bombel_msgs.msg import BombelSpeed

import tf
from math import *

BOMBEL_COM_RATE = 100.0
NODE_NAME = "TEST_PATH"
NODE_NAME_HEADER = "[" + NODE_NAME + "]"

acc = 50.0 # unit/s
speed = 0.0

bombelSpeedMsg = BombelSpeed()


if __name__ == '__main__':
	rospy.init_node(NODE_NAME, anonymous=True)

	pub = rospy.Publisher('/bombel/speed', BombelSpeed, queue_size=10)
	# rospy.Subscriber("/joint_states", JointState, msg_received)
	
	rate = rospy.Rate(BOMBEL_COM_RATE) # 10hz

	bombelSpeedMsg.joint0_speed = 0
	bombelSpeedMsg.joint1_speed = 0
	bombelSpeedMsg.joint2_speed = 0

	print bombelSpeedMsg
	rospy.loginfo(NODE_NAME_HEADER + "Init ok")

	i=0
	while not rospy.is_shutdown():

		if( i%BOMBEL_COM_RATE == 0):
			rospy.loginfo("SPEED: " + str(speed))

		pub.publish(bombelSpeedMsg)
		
		speed += acc / BOMBEL_COM_RATE
		bombelSpeedMsg.joint0_speed = speed;
		if(speed >= 400.0 or speed <= 0.0):
			 acc = -acc
			 rospy.loginfo("CHANGING DIRECTION")

		i+=1
		rate.sleep()

