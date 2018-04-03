#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():

	demo_publisher = rospy.Publisher("DEMO", String, queue_size = 10)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		DEMO = input("Enter the formation you would like to see : ")
		demo_publisher.publish(DEMO)
		rate.sleep()

if __name__== '__main__':

	rospy.init_node("user_input",anonymous = True)
	try:
		talker()
	except rospy.ROSInterruptException:
		pass