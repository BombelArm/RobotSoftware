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
		

ikin_server=0
dikin_server=0
posePublisher=0
pathPublisher=0
bombelPosPub=0

timeOfExecution = 10
loopRate = 10
pos1 = Point(0.0, 0.0 ,0.0)
pos2 = Point(0.4, 0.0 ,0.2)
pos3 = Point(0.3, 0.15 ,0.2)

jointStateMsg = JointState()
pathMsg = Path()
bombelPos = BombelPos();
seq=1;

def calculateIkin(point):
    try:
    	req = BombelIkinRequest()
    	req.point = point
        resp = ikin_server(req)
        return resp.jointState
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def calculateDikin(jointState):
    try:
    	req = BombelDikinRequest()
    	req.jointState = jointState
        resp = dikin_server(req)
        return resp.point
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def calculatePoly(x,ox, ta, t):
  result = 0.1
  result  = ((-2.0*(x-ox))/(t*t*t))*(ta*ta*ta) + ((3.0*(x-ox))/(t*t))*(ta*ta) + ox; 
  return result 

def interpolate(startPos, endPos, timeOfExecution, loopRate):
	global posePublisher, statePublisher, pathPublisher, bombelPosPub, pathMsg, seq

	poseStampedMsg = PoseStamped()
	frameId = "base_link"
	poseStampedMsg.header.frame_id=frameId
	pathMsg.header.frame_id = frameId

	currentPosition = Point()
                                                                               
	rate = rospy.Rate(loopRate)

	currentPosition = copy.deepcopy(startPos)
	# currentPosition = startPos
	steps = (loopRate * timeOfExecution)+1
	rx = (endPos.x - startPos.x)/steps
	ry = (endPos.y - startPos.y)/steps
	rz = (endPos.z - startPos.z)/steps
	rospy.loginfo("Starting interpolate")

	timeNow = 0
	for i in range(steps):
		poseStampedMsg.header.stamp = rospy.Time.now()
		pathMsg.header.stamp = rospy.Time.now()
		jointStateMsg.header.stamp = rospy.Time.now()

	
		# poseStampedMsg.pose.position.x = calculatePoly(endPos.x, currentPosition.x, timeNow, timeOfExecution)
		# poseStampedMsg.pose.position.y = calculatePoly(endPos.y, currentPosition.y, timeNow, timeOfExecution)
		# poseStampedMsg.pose.position.z = calculatePoly(endPos.z, currentPosition.z, timeNow, timeOfExecution)

		# rx = (endPos.x - startPos.x)/steps
		# ry = (endPos.y - startPos.y)/steps
		# rz = (endPos.z - startPos.z)/steps


		# currentPosition = poseStampedMsg.pose.position
		currentPosition.x += rx
		currentPosition.y += ry
		currentPosition.z += rz
		
		poseStampedMsg.pose.position = currentPosition


		timeNow += 1.0/(loopRate)
		# print str(timeNow) + '/' + str(timeOfExecution)

		jointStates = calculateIkin(currentPosition)
		if not jointStates:
			jointStates = [0, 0, 0]

		pathPose = copy.deepcopy(poseStampedMsg)
		pathMsg.poses.append(pathPose)

		bombelPos.seq = seq
		bombelPos.joint0_pos = jointStates[0]
		bombelPos.joint1_pos = jointStates[1]
		bombelPos.joint2_pos = jointStates[2]
		seq += 1

		jointStateMsg.position = jointStates	

		print bombelPos
		# posePublisher.publish(poseStampedMsg)
		# statePublisher.publish(jointStateMsg)
		bombelPosPub.publish(bombelPos)
		pathPublisher.publish(pathMsg)

		rate.sleep()
	bombelPos.seq = -1;
	bombelPosPub.publish(bombelPos)

	rospy.loginfo("Generation completed")
	rospy.loginfo(currentPosition)
	rospy.loginfo(endPos)

if __name__ == '__main__':
	msg=PoseStamped()

	rospy.init_node('generator', anonymous=True)
	posePublisher = rospy.Publisher('/generator/pose', PoseStamped, queue_size=10)
	pathPublisher = rospy.Publisher('generator/path',Path, queue_size=10)
	bombelPosPub = rospy.Publisher('/bombel/pos',BombelPos, queue_size=10)
	statePublisher = rospy.Publisher('joint_states', JointState, queue_size=10)
	# # rospy.Subscriber("/joint_states", JointState, msg_received)
	
	rospy.wait_for_service('bombel/ikin_server')
	rospy.wait_for_service('bombel/dikin_server')
	ikin_server = rospy.ServiceProxy('bombel/ikin_server', BombelIkin)
	dikin_server = rospy.ServiceProxy('bombel/dikin_server', BombelDikin)

	jointStateMsg.name = ["joint0", "joint1", "joint2"]
	jointStateMsg.position = [0.0, 0.0, 0.0]
	pos1 = calculateDikin(jointStateMsg.position);

	rospy.loginfo("[Ikin_Client] Init OK!")

	interpolate(pos1, pos2, timeOfExecution, loopRate)
	# interpolate(pos2, pos1, timeOfExecution, loopRate)

	
	rospy.signal_shutdown("Closing generator ... ")		


		
