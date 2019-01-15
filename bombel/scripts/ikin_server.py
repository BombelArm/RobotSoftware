#!/usr/bin/env python

import rospy

from std_msgs.msg import *

from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import JointState
from bombel_msgs.srv import *

import tf
from math import *



loopRate = 100

theta1_lower=0
theta1_upper=0
theta2_lower=0
theta2_upper=0
theta3_lower=0
theta3_upper=0

d1=0
a2=0
a3=0
a4=0
theta4=0

def calc_ikin(req):
	global pub, d1, a2, a3, a4, theta4, theta1_lower, theta1_upper, theta2_lower, theta2_upper, theta3_lower, theta3_upper

	response = BombelIkinResponse()
	response.jointState = []
	x=req.point.x
	y=req.point.y
	z=req.point.z

	beta=atan2(a4*sin(theta4),a3+a4*cos(theta4))
	k=a4*sin(pi-theta4)/sin(beta)

	if (sqrt(x*x+y*y+(z-d1)*(z-d1)) >= a2+k):
		return response

	cos3=(x*x+y*y+(z-d1)*(z-d1)-a2*a2-k*k)/(2*a2*k)
	sin3=sqrt(1-pow(cos3,2))
	theta3_prim=atan2(sin3,cos3)

	theta1=atan2(y,x)
	theta2=atan2(sqrt(x*x+y*y),z-d1)-atan2(k*sin(theta3_prim),a2+k*cos(theta3_prim))
	theta3=theta3_prim-beta

	# if theta1>theta1_upper or theta1<theta1_lower :
	# 	return pointResponse("Theta1 violation"+str(theta1))
		
	if theta2>theta2_upper or theta2<theta2_lower :
		# Theta2 violation
		return response
		
	if theta3>theta3_upper or theta3<theta3_lower :
		# Theta3 violation
		return response

	response.jointState = [theta1, theta2, theta3]

	return response


def get_params():
	global d1, a2, a3, a4, theta4, theta1_lower, theta1_upper, theta2_lower, theta2_upper, theta3_lower, theta3_upper

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

	if rospy.has_param('theta1_lower'):
		theta1_lower = rospy.get_param("theta1_lower")
	else:
		print "No theta1_lower param"+"\n"
		rospy.signal_shutdown("No theta1_lower param")			

	if rospy.has_param('theta1_upper'):
		theta1_upper = rospy.get_param("theta1_upper")
	else:
		print "No theta1_upper param"+"\n"
		rospy.signal_shutdown("No theta1_upper param")

	if rospy.has_param('theta2_lower'):
		theta2_lower = rospy.get_param("theta2_lower")
	else:
		print "No theta2_lower param"+"\n"
		rospy.signal_shutdown("No theta2_lower param")			

	if rospy.has_param('theta2_upper'):
		theta2_upper = rospy.get_param("theta2_upper")
	else:
		print "No theta2_upper param"+"\n"
		rospy.signal_shutdown("No theta2_upper param")

	if rospy.has_param('theta3_lower'):
		theta3_lower = rospy.get_param("theta3_lower")
	else:
		print "No theta3_lower param"+"\n"
		rospy.signal_shutdown("No theta3_lower param")			

	if rospy.has_param('theta3_upper'):
		theta3_upper = rospy.get_param("theta3_upper")
	else:
		print "No theta3_upper param"+"\n"
		rospy.signal_shutdown("No theta3_upper param")

if __name__ == '__main__':
	rospy.init_node('ikin_server', anonymous=True)
	get_params()

	s = rospy.Service('bombel/ikin_server', BombelIkin, calc_ikin)

	rate = rospy.Rate(loopRate) # 10hz
	rospy.loginfo("[Ikin_Server] Init OK!")

	while not rospy.is_shutdown():
		rate.sleep()

