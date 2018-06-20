#!/usr/bin/env python

import rospy

from std_msgs.msg import *

from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from bombel.srv import *
		

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
	pub = rospy.Publisher('/bombel_ikin', PoseStamped, queue_size=100)
	# rospy.Subscriber("/joint_states", JointState, msg_received)
	
	rospy.wait_for_service('calc_ikin')
	server = rospy.ServiceProxy('calc_ikin', point)
	rate = rospy.Rate(10) # 10hz

	rospy.loginfo("[Ikin_Client] Init OK!")


	msg.header.frame_id="base_link"

	while not rospy.is_shutdown():
		print "Pass [x,y,z]:\n"

		try:
			x=float(raw_input("X:"))
			y=float(raw_input("Y:"))
			z=float(raw_input("Z:"))

			msg.header.stamp=rospy.Time.now()
			msg.pose.position.x=x
			msg.pose.position.y=y
			msg.pose.position.z=z

			pub.publish(msg)

			res=calc_ikin(x,y,z)

			rospy.loginfo(res)
		except Exception, e:
			rospy.signal_shutdown("Keybord interrupt")
			print e

		rate.sleep()


		
