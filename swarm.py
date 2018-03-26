#!/usr/bin/env python
import rospy
import time
from geometry_msgs.msg import Point32

robot_xy = ['']
goal_xy =['']
goal_publishers = {}

def robot_locations(data,robot_number):
	global robot_xy
	robot_xy[robot_number] = [data.x,data.y]

def talker():
	global robot_xy

	rate = rospy.Rate(10)

	for i in range(number_of_robots):
		rospy.Subscriber("/location/robot"+str(i+1), Point32, robot_locations,i)
		goal_publishers["pub{0}".format(i+1)] = rospy.Publisher("goal/robot"+str(i+1), Point32, queue_size = 10)

	## SWARM MATH GOES HERE


	
	while not rospy.is_shutdown():

		#print("swarm_brain | ", robot_xy)
		
		## GOAL TRANSMISSION
		for i in range(number_of_robots):
			p = Point32()
			p.x = goal_xy[i][0]
			p.y = goal_xy[i][1]
			goal_publishers["pub"+str(i+1)].publish(p)
		
		rate.sleep()	

if __name__== '__main__':

	rospy.init_node("swarm",anonymous = True)
	try:
		init = (rospy.get_param("init"))
		connections = [init[a] for a in init]
		number_of_robots = len(connections)
		robot_xy = ['']*number_of_robots
		goal_xy =[[50.0,50.0]]*5
		talker()
	except rospy.ROSInterruptException:
		pass