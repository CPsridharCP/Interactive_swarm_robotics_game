#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():

	accepted = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
	demo_publisher = rospy.Publisher("DEMO", String, queue_size = 10)
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		valid = True
		print ("")
		DEMO = input("Enter the formation you would like to see : ")
		for x in DEMO:
			if x not in accepted:
				print ("Please enter characters without space in caps !!")
				print ("")
				valid = False
		if valid:		
			demo_publisher.publish(DEMO)
		rate.sleep()

if __name__== '__main__':


	rospy.init_node("user_input",anonymous = True)
	try:
		talker()
	except rospy.ROSInterruptException:
		pass