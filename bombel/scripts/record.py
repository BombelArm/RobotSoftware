#!/usr/bin/env python

import rospy
import signal
import sys

from std_msgs.msg import *

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from bombel_msgs.msg import *
from bombel_msgs.srv import *


fPoseReal = None
fPoseGen = None
rate = None

columns = "time\tpx\tpy\tpz\tox\toy\toz\tow\n"

def sigint_handler(sig, frame):
	global fPoseReal, fPoseGen

	fPoseReal.close()
	# fPoseGen.close()
	rospy.loginfo("Recorder end")

	sys.exit(0)

def poseReal(poseStamped):
	global fPoseReal

	stamp = poseStamped.header.stamp
	position = poseStamped.pose.position
	orientation = poseStamped.pose.orientation
	fPoseReal.write("%s.%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (stamp.secs, stamp.nsecs, position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w))


def poseGen(poseStamped):
	global fPoseGen

	stamp = poseStamped.header.stamp
	position = poseStamped.pose.position
	orientation = poseStamped.pose.orientation
	fPoseGen.write("%s.%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n" % (stamp.secs, stamp.nsecs, position.x, position.y, position.z, orientation.x, orientation.y, orientation.z, orientation.w))

if __name__ == '__main__':	

	signal.signal(signal.SIGINT, sigint_handler)
	rospy.init_node('recorder', anonymous=True)


	rospy.Subscriber("/bombel/pose_real", PoseStamped, poseReal)
	# rospy.Subscriber("/generator/pose", PoseStamped, poseGen)	

	fPoseReal = open("./trace/poseReal.txt","w+")
	# fPoseGen = open("./trace/poseGen.txt","w+")

	fPoseReal.write(columns)
	# fPoseGen.write(columns)

	rospy.loginfo("Recorder started")

	rospy.spin()

	fPoseReal.close()
	# fPoseGen.close()

