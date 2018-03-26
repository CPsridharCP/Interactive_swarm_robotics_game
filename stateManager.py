#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Polygon,Point32

def talker():
	pub = rospy.Publisher('LED_Contols', Polygon, queue_size = 10)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():

		# Bluetooth Communication Check
		if not (rospy.get_param("connection_established")):

			try:
				init = (rospy.get_param("init"))
			except:
				continue
			connections = [init[a] for a in init]
			if 0 in connections:
				pass
			else:
				rospy.set_param("connection_established", True)
				print ("STATE MANAGER :: Bluetooth Communication Established to all Robots")


		# Initial Localisation
		if (rospy.get_param("connection_established")) and not (rospy.get_param("all_robot_localised")):
			try:
				locate = (rospy.get_param("locate"))
			except:
				continue
			initial_location = [locate[a] for a in locate]
			if 0 in initial_location:
				pass
			else:
				rospy.set_param("all_robot_localised", True)
				print ("STATE MANAGER :: All Robots Localised")
				print ("STATE MANAGER :: Continuous Tracking ON")

		rate.sleep()


if __name__== '__main__':
	try:
		rospy.init_node("stateManager",anonymous = True)
		talker()
	except rospy.ROSInterruptException:
		pass