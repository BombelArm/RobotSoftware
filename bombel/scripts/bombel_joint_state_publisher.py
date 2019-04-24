#!/usr/bin/env python

import rospy

from std_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from bombel_msgs.msg import BombelState

import tf
from math import *




posePublisher=0
robotStatePublisher=0
seq=0

d1=0
a2=0
a3=0
a4=0
theta3=0

def msg_received(data):
	global posePublisher, robotStatePublisher, d1, a2, a3, a4, theta3, seq

	encoder0=data.encoder0_pos
	encoder1=data.encoder1_pos
	encoder2=data.encoder2_pos

	theta0 = (encoder0 / pow(2,11)) * pi
	theta1 = (encoder1 / pow(2,11)) * pi
	theta2 = (encoder2 / pow(2,11)) * pi

	poseMsg=PoseStamped()
	jointMsg = JointState()

	poseMsg.header.frame_id="base_link"
	poseMsg.header.stamp=rospy.Time.now()

	const_xy=a2*sin(theta1)+a3*sin(theta1+theta2)+a4*sin(theta1+theta2+pi/4)
	poseMsg.pose.position.x=cos(theta0)*const_xy
	poseMsg.pose.position.y=sin(theta0)*const_xy
	poseMsg.pose.position.z=d1+a2*cos(theta1)+a3*cos(theta1+theta2)+a4*cos(theta1+theta2+theta3)

	quaternion = tf.transformations.quaternion_from_euler(0, theta1+theta2-theta3, theta0)
	poseMsg.pose.orientation.x=quaternion[0]
	poseMsg.pose.orientation.y=quaternion[1]
	poseMsg.pose.orientation.z=quaternion[2]
	poseMsg.pose.orientation.w=quaternion[3]

	jointMsg.header.seq = seq
	seq += 1
	jointMsg.header.stamp = rospy.Time().now()

	jointMsg.name = ['joint0', 'joint1', 'joint2']
	jointMsg.position= [theta0, theta1, theta2]


	# posePublisher.publish(poseMsg)
	robotStatePublisher.publish(jointMsg)


def get_params():
	global d1, a2, a3, a4, theta3

	if rospy.has_param('d1'):
		d1 = rospy.get_param("d1")
	else:
		print "No d1 param"+"\n"
		rospy.signal_shutdown("No d1 param")

	if rospy.has_param('a2'):
		a2 = rospy.get_param("a2")
	else:
		print "No a2 param"+"\n"
		rospy.signal_shutdown("No a2 param")

	if rospy.has_param('a3'):
		a3 = rospy.get_param("a3")
	else:
		print "No a3 param"+"\n"
		rospy.signal_shutdown("No a3 param")

	if rospy.has_param('a4'):
		a4 = rospy.get_param("a4")
	else:
		print "No a4 param"+"\n"
		rospy.signal_shutdown("No a4 param")
	
	if rospy.has_param('theta3'):
		theta3 = rospy.get_param("theta3")
	else:
		print "No theta3 param"+"\n"
		rospy.signal_shutdown("No theta3 param")		

if __name__ == '__main__':
	rospy.init_node('BombelJointStatePublisher', anonymous=True)
	get_params()

	posePublisher = rospy.Publisher('/bombel_dkin', PoseStamped, queue_size=100)
	robotStatePublisher = rospy.Publisher('/joint_states', JointState, queue_size=100)
	rospy.Subscriber("/bombel/state", BombelState, msg_received)

	rate = rospy.Rate(10) # 10hz

	rospy.spin()
