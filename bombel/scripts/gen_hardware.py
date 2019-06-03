#!/usr/bin/env python

import rospy
from math import *

from std_msgs.msg import *

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from bombel_msgs.msg import *
from bombel_msgs.srv import *

from bombel_msgs import BombelCmdType
BombelCmdType = BombelCmdType()
		

ikin_server = None
dikin_server = None

posePublisher = None
pathPublisher = None
bombelCmdPub = None

timeOfExecution = 15
loopRate = 20

# pos1 = Point(0.0, 0.0 ,0.0)
# pos2 = Point(0.3, 0.2 ,0.2)
# pos3 = Point(0.3, -0.2 ,0.2)


pos1 = Point(0.25, 0.1 ,0.4)
pos2 = Point(0.25, 0.1 ,0.2)
pos3 = Point(0.25, -0.1 ,0.2)
pos4 = Point(0.25, -0.1 ,0.4)
pos5 = Point(0.25, 0.1 ,0.4)


pos6 = Point(0.25, -0.1 ,0.3)
pos7 = Point(0.25, 0.1 ,0.2)


pathMsg = Path()

def calculateIkin(point):
    try:
    	req = BombelIkinRequest()
    	req.point = point
        resp = ikin_server(req)
        return resp.jointState
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def calculateDkin(jointState):
    try:
    	req = BombelDikinRequest()
    	req.jointState = jointState
        resp = dikin_server(req)
        return resp.point
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def calculatePoly(x,ox, ta, t):
  result  = ((-2.0*(x-ox))/(t*t*t))*(ta*ta*ta) + ((3.0*(x-ox))/(t*t))*(ta*ta) + ox; 
  return result 

def interpolatePosition(startPos, endPos, timeOfExecution, loopRate):
	global pathMsg
	
	bombelCmd = BombelCmd()

	poseStampedMsg = PoseStamped()
	poseStampedMsg.header.frame_id= "base_link"

	pathMsg.header.frame_id = "base_link"
	nextPosition = Point()

	timeNow = 0.0
	seq = 0
	rate =rospy.Rate(loopRate)


	bombelCmd.cmd = BombelCmdType.Start() #start 
	bombelCmdPub.publish(bombelCmd)

	while(not rospy.is_shutdown()):
		timestamp = rospy.Time.now()

		#incrementing time for calculatePoly fun

		# calculating next pos
		nextPosition.x = calculatePoly(endPos.x, startPos.x, timeNow, timeOfExecution)
		nextPosition.y = calculatePoly(endPos.y, startPos.y, timeNow, timeOfExecution)
		nextPosition.z = calculatePoly(endPos.z, startPos.z, timeNow, timeOfExecution)

		# calculating inverse kinematics
		nextJointState = calculateIkin(nextPosition)

		# sending PoseStamped to Rviz
		poseStampedMsg.header.stamp = timestamp
		poseStampedMsg.pose.position = nextPosition
		posePublisher.publish(poseStampedMsg)

		#sending Path to Rviz 
		pathMsg.header.stamp = timestamp
		pathMsg.poses.append(poseStampedMsg)
		pathPublisher.publish(pathMsg)

		#sending cmd to Bombel
		bombelCmd.seq = seq
		bombelCmd.cmd = BombelCmdType.SetNextPosition() #setNextPos
		bombelCmd.joint0_pos = nextJointState[0]
		bombelCmd.joint1_pos = nextJointState[1]
		bombelCmd.joint2_pos = nextJointState[2]
		bombelCmdPub.publish(bombelCmd)

		print "Pos: x:{0} y:{1} z:{2}\ttime {3}".format(nextPosition.x, nextPosition.y, nextPosition.z,timeNow)

		seq += 1
		timeNow = timeNow + 1.0/loopRate
		if(timeNow >= timeOfExecution):
			break;
		rate.sleep()

	#stopping bombel
	bombelCmd.cmd = BombelCmdType.HardStop()
	bombelCmdPub.publish(bombelCmd)
	print "Interpolation from {0} to {1} ended.".format(startPos,endPos)


if __name__ == '__main__':
	rospy.init_node('generator', anonymous=True)
	posePublisher = rospy.Publisher('/generator/pose', PoseStamped, queue_size=10)
	pathPublisher = rospy.Publisher('/generator/path',Path, queue_size=10)
	bombelCmdPub = rospy.Publisher('/bombel/cmd',BombelCmd, queue_size=10)
	
	rospy.wait_for_service('bombel/ikin_server')
	rospy.wait_for_service('bombel/dikin_server')
	ikin_server = rospy.ServiceProxy('bombel/ikin_server', BombelIkin)
	dikin_server = rospy.ServiceProxy('bombel/dikin_server', BombelDikin)
	rate = rospy.Rate(loopRate)

	#setting up messages
	pos0 = calculateDkin([0.0, 0.0, 0.0]);

	poseStampedMsg = PoseStamped()
	
	rospy.loginfo("Hardware generator ready!")
	# base(timeOfExecution*2,loopRate)
	
	# interpolatePosition(pos1,pos2,timeOfExecution,loopRate)
	# interpolatePosition(pos2,pos3,timeOfExecution,loopRate)
	# interpolatePosition(pos3,pos1,timeOfExecution,loopRate)


	interpolatePosition(pos0,pos1,timeOfExecution,loopRate)
	rospy.sleep(2.)
	interpolatePosition(pos1,pos2,timeOfExecution,loopRate)
	interpolatePosition(pos2,pos3,timeOfExecution,loopRate)
	interpolatePosition(pos3,pos4,timeOfExecution,loopRate)
	interpolatePosition(pos4,pos5,timeOfExecution,loopRate)
	interpolatePosition(pos5,pos1,timeOfExecution,loopRate)

	rospy.sleep(2.)

	interpolatePosition(pos1,pos6,timeOfExecution,loopRate)
	interpolatePosition(pos6,pos7,timeOfExecution,loopRate)
	interpolatePosition(pos7,pos1,timeOfExecution,loopRate)

	# while not rospy.is_shutdown():
	# 	pathPublisher.publish(pathMsg)
		
