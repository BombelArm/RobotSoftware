#!/usr/bin/env python

import rospy

from std_msgs.msg import *
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from bombel_msgs.srv import *

import tf
from math import *



loopRate = 100

d1=0
a2=0
a3=0
a4=0
theta3=0

def calc_dikin(req):
	global  d1, a2, a3, a4, theta3, seq

	response = BombelDikinResponse()
	
	theta0 = req.jointState[0]
	theta1 = req.jointState[1]
	theta2 = req.jointState[2]

	const_xy=a2*sin(theta1)+a3*sin(theta1+theta2)+a4*sin(theta1+theta2+pi/4)
	response.point.x=cos(theta0)*const_xy
	response.point.y=sin(theta0)*const_xy
	response.point.z=d1+a2*cos(theta1)+a3*cos(theta1+theta2)+a4*cos(theta1+theta2+theta3)

	# quaternion = tf.transformations.quaternion_from_euler(0, theta1+theta2-theta3, theta0)
	# poseMsg.pose.orientation.x=quaternion[0]
	# poseMsg.pose.orientation.y=quaternion[1]
	# poseMsg.pose.orientation.z=quaternion[2]
	# poseMsg.pose.orientation.w=quaternion[3]

	return response;


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
	rospy.init_node('ikin_server', anonymous=True)
	get_params()

	s = rospy.Service('bombel/dikin_server', BombelDikin, calc_dikin)

	rate = rospy.Rate(loopRate) # 10hz
	rospy.loginfo("[Dikin_Server] Init OK!")

	while not rospy.is_shutdown():
		rate.sleep()

