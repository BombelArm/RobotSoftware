#!/usr/bin/env python

import rospy

from std_msgs.msg import *

from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import JointState
from bombel.srv import *

import tf
from math import *



pub=0

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

jState=JointState()

def calc_ikin(req):
	global pub, d1, a2, a3, a4, theta4, jState, theta1_lower, theta1_upper, theta2_lower, theta2_upper, theta3_lower, theta3_upper



	x=req.x
	y=req.y
	z=req.z

	beta=atan2(a4*sin(theta4),a3+a4*cos(theta4))
	k=a4*sin(pi-theta4)/sin(beta)

	# cos3_prim=(pow(x,2)+pow(y,2)+pow(z-d1,2)-pow(a2,2)-pow(k,2))/2*a2*k
	# sin3_prim=sqrt(1-pow(cos3_prim,2))
	# theta3_prim=atan2(sin3_prim,cos3_prim)

	if (sqrt(x*x+y*y+(z-d1)*(z-d1)) >= a2+k):
		return pointResponse("Fault coordinates")


	cos3=(x*x+y*y+(z-d1)*(z-d1)-a2*a2-k*k)/(2*a2*k)
	sin3=sqrt(1-pow(cos3,2))
	theta3_prim=atan2(sin3,cos3)

	theta1=atan2(y,x)
	theta2=atan2(sqrt(x*x+y*y),z-d1)-atan2(k*sin(theta3_prim),a2+k*cos(theta3_prim))
	theta3=theta3_prim-beta

	# if theta1>theta1_upper or theta1<theta1_lower :
	# 	return pointResponse("Theta1 violation"+str(theta1))
		
	if theta2>theta2_upper or theta2<theta2_lower :
		return pointResponse("Theta2 violation"+str(theta2))
		
	if theta3>theta3_upper or theta3<theta3_lower :
		return pointResponse("Theta3 violation"+str(theta3))
		

	jState.position[0]=theta1
	jState.position[1]=theta2
	jState.position[2]=theta3

	return pointResponse("[IKIN_SRV] Service called properly \n")


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

	pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
	# rospy.Subscriber("/joint_states", JointState, msg_received)
	s = rospy.Service('calc_ikin', point, calc_ikin)

	rate = rospy.Rate(10) # 10hz
	rospy.loginfo("[Ikin_Server] Init OK!")

	jState.name=["joint0","joint1", "joint2"]
	jState.position=[0.0, 0.0, 0.0]

	while not rospy.is_shutdown():
		jState.header.stamp=rospy.Time.now()
		pub.publish(jState)
		rate.sleep()

