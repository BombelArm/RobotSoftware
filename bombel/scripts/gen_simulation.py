#!/usr/bin/env python

import rospy
import copy

from std_msgs.msg import *

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from sensor_msgs.msg import JointState
from bombel_msgs.msg import *
from bombel_msgs.srv import *
		

ikin_server = None
dikin_server = None

posePublisher = None
pathPublisher = None
bombelPosPub = None
jointStatePublisher = None

timeOfExecution = 5
loopRate = 20

pos1 = Point(0.0, 0.0 ,0.0)
pos2 = Point(0.3, 0.2 ,0.2)
pos3 = Point(0.3, -0.2 ,0.2)

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

def interpolate(startPos, endPos, timeOfExecution, loopRate):
	poseStampedMsg = PoseStamped()
	poseStampedMsg.header.frame_id= "base_link"
	poseStampedMsg.pose.position = copy.deepcopy(startPos)

	jointStateMsg = JointState()
	jointStateMsg.name = ["joint0", "joint1", "joint2"]
	jointStateMsg.position = [0.0, 0.0, 0.0]

	pathMsg = Path()
	pathMsg.header.frame_id = "base_link"

	nextPosition = Point()

	timeNow = 0.0
	rate =rospy.Rate(loopRate)

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

		#sending JointState to Rviz
		jointStateMsg.header.stamp = timestamp
		jointStateMsg.position = nextJointState
		jointStatePublisher.publish(jointStateMsg)

		print "Pos: x:{0} y:{1} z:{2}\ttime {3}".format(nextPosition.x, nextPosition.y, nextPosition.z,timeNow)

		timeNow = timeNow + 1.0/loopRate
		if(timeNow >= timeOfExecution):
			break;
		rate.sleep()

	print "Interpolation from {0} to {1} ended.".format(startPos,endPos)

if __name__ == '__main__':
	rospy.init_node('generator', anonymous=True)
	posePublisher = rospy.Publisher('/generator/pose', PoseStamped, queue_size=10)
	pathPublisher = rospy.Publisher('/generator/path',Path, queue_size=10)
	bombelPosPub = rospy.Publisher('/bombel/pos',BombelPos, queue_size=10)
	jointStatePublisher = rospy.Publisher('joint_states', JointState, queue_size=10)
	
	rospy.wait_for_service('bombel/ikin_server')
	rospy.wait_for_service('bombel/dikin_server')
	ikin_server = rospy.ServiceProxy('bombel/ikin_server', BombelIkin)
	dikin_server = rospy.ServiceProxy('bombel/dikin_server', BombelDikin)
	rate = rospy.Rate(loopRate)

	#setting up messages
	pos1 = calculateDkin([0.0, 0.0, 0.0]);

	poseStampedMsg = PoseStamped()

    
	# currentPosition = copy.deepcopy(startPos)

	rospy.loginfo("[Ikin_Client] Init OK!")

	interpolate(pos1,pos2,timeOfExecution,loopRate)
	interpolate(pos2,pos3,timeOfExecution,loopRate)
	interpolate(pos3,pos1,timeOfExecution,loopRate)