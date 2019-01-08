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

acc = 50.0 # unit/s
speed = 0.0

bombelSpeedMsg = BombelSpeed()
bombelStateMsg = BombelState()

if __name__ == '__main__':
	rospy.init_node(NODE_NAME, anonymous=True)

	pub = rospy.Publisher('/bombel/state_test', BombelState, queue_size=10)
	# rospy.Subscriber("/joint_states", JointState, msg_received)
	
	rate = rospy.Rate(BOMBEL_COM_RATE) # 10hz

	bombelStateMsg.encoder0_pos = -10
	bombelStateMsg.encoder1_pos = 10
	bombelStateMsg.encoder2_pos = -20

	rospy.loginfo(NODE_NAME_HEADER + "Init ok")

	i=0
	while not rospy.is_shutdown():
		pub.publish(bombelStateMsg)
		i+=1
		rate.sleep()

