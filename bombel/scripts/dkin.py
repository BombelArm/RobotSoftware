#!/usr/bin/env python

import rospy

from std_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

import tf
from math import *




pub=0

d1=0
a2=0
a3=0
a4=0
theta4=0

def msg_received(data):
	global pub, d1, a2, a3, a4, theta4


	theta1=data.position[0]
	theta2=data.position[1]
	theta3=data.position[2]

	msg=PoseStamped()
	msg.header.frame_id="base_link"
	msg.header.stamp=rospy.Time.now()

	const_xy=a2*sin(theta2)+a3*sin(theta2+theta3)+a4*sin(theta2+theta3+pi/4)
	msg.pose.position.x=cos(theta1)*const_xy
	msg.pose.position.y=sin(theta1)*const_xy
	msg.pose.position.z=d1+a2*cos(theta2)+a3*cos(theta2+theta3)+a4*cos(theta2+theta3+theta4)

	quaternion = tf.transformations.quaternion_from_euler(0, theta2+theta3-theta4, theta1)
	msg.pose.orientation.x=quaternion[0]
	msg.pose.orientation.y=quaternion[1]
	msg.pose.orientation.z=quaternion[2]
	msg.pose.orientation.w=quaternion[3]

	pub.publish(msg)


def get_params():
	global d1, a2, a3, a4, theta4

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
	
	if rospy.has_param('theta4'):
		theta4 = rospy.get_param("theta4")
	else:
		print "No theta4 param"+"\n"
		rospy.signal_shutdown("No theta4 param")		

if __name__ == '__main__':
	rospy.init_node('dkin', anonymous=True)
	get_params()

	pub = rospy.Publisher('/bombel_dkin', PoseStamped, queue_size=100)
	rospy.Subscriber("/joint_states", JointState, msg_received)

	rate = rospy.Rate(10) # 10hz

	rospy.spin()
