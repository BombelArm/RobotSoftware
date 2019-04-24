#!/usr/bin/env python

import rospy

from std_msgs.msg import *

from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import JointState
from bombel_msgs.srv import *

import tf
from math import *



loopRate = 100

theta0_lower=0
theta0_upper=0
theta1_lower=0
theta1_upper=0
theta2_lower=0
theta2_upper=0

d1=0
a2=0
a3=0
a4=0
theta3=0

def calc_ikin(req):
	global pub, d1, a2, a3, a4, theta3, theta0_lower, theta0_upper, theta1_lower, theta1_upper, theta2_lower, theta2_upper

	response = BombelIkinResponse()
	response.jointState = []
	x=req.point.x
	y=req.point.y
	z=req.point.z

	beta=atan2(a4*sin(theta3),a3+a4*cos(theta3))
	k=a4*sin(pi-theta3)/sin(beta)

	if (sqrt(x*x+y*y+(z-d1)*(z-d1)) >= a2+k):
		return response

	cos2=(x*x+y*y+(z-d1)*(z-d1)-a2*a2-k*k)/(2*a2*k)
	sin2=sqrt(1-pow(cos2,2))
	theta2_prim=atan2(sin2,cos2)

	theta0=atan2(y,x)
	theta1=atan2(sqrt(x*x+y*y),z-d1)-atan2(k*sin(theta2_prim),a2+k*cos(theta2_prim))
	theta2=theta2_prim-beta

	# if theta0>theta0_upper or theta0<theta0_lower :
	# 	return pointResponse("Theta0 violation"+str(theta0))
		
	if theta1>theta1_upper or theta1<theta1_lower :
		# Theta1 violation
		return response
		
	if theta2>theta2_upper or theta2<theta2_lower :
		# Theta2 violation
		return response

	response.jointState = [theta0, theta1, theta2]

	return response


def get_params():
	global d1, a2, a3, a4, theta3, theta0_lower, theta0_upper, theta1_lower, theta1_upper, theta2_lower, theta2_upper

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

	if rospy.has_param('theta0_lower'):
		theta0_lower = rospy.get_param("theta0_lower")
	else:
		print "No theta0_lower param"+"\n"
		rospy.signal_shutdown("No theta0_lower param")			

	if rospy.has_param('theta0_upper'):
		theta0_upper = rospy.get_param("theta0_upper")
	else:
		print "No theta0_upper param"+"\n"
		rospy.signal_shutdown("No theta0_upper param")

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

if __name__ == '__main__':
	rospy.init_node('ikin_server', anonymous=True)
	get_params()

	s = rospy.Service('bombel/ikin_server', BombelIkin, calc_ikin)

	rate = rospy.Rate(loopRate) # 10hz
	rospy.loginfo("[Ikin_Server] Init OK!")

	while not rospy.is_shutdown():
		rate.sleep()

