#!/usr/bin/env python

import rospy

from std_msgs.msg import *

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from bombel.srv import *
		

loopRate = 100
server=0

def calc_ikin(x, y,z):
    try:
        resp1 = server(x, y, z)
        return resp1.status
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



if __name__ == '__main__':
	msg=PoseStamped()

	rospy.init_node('ikin_client', anonymous=True)
	rospy.wait_for_service('calc_ikin')
	server = rospy.ServiceProxy('calc_ikin', point)
	rate = rospy.Rate(loopRate) # 10hz

	rospy.loginfo("[Ikin_Client] Init OK!")


	msg.header.frame_id="base_link"

	while not rospy.is_shutdown():
		rate.sleep()


		
