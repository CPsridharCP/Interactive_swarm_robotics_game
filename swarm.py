#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Point32
from math import cos,sin,radians
from scipy.spatial import distance
from scipy.optimize import linear_sum_assignment
import numpy as np

robot_xy = ['']
goal_xy =['']
goal_publishers = {}

angle = 0

corner_centre = [[20.0,20.0],[80.0,80.0],[20.0,80.0],[80.0,20.0],[50.0,50.0]]

def map_to_pix(x,y):
	x = x*7
	y = (50 + (50-y))*7
	return x,y

def map_to_xy(x,y):
	x = int(x//7)
	y = 50 + (50-(int(y//7)))
	return x,y

def robot_locations(data,robot_number):
	global robot_xy
	robot_xy[robot_number] = [data.x,data.y]

def talker():
	global robot_xy,angle,goal_xy,corner_centre

	rate = rospy.Rate(10)

	for i in range(number_of_robots):
		rospy.Subscriber("/location/robot"+str(i+1), Point32, robot_locations,i)
		goal_publishers["pub{0}".format(i+1)] = rospy.Publisher("goal/robot"+str(i+1), Point32, queue_size = 10)

	## SWARM MATH GOES HERE

	while not rospy.is_shutdown():

		#print("swarm_brain | ", robot_xy)

		#rad =25
		#theta = radians(angle%360)
		#x_g = int(rad*cos(theta))+50
		#y_g = int(rad*sin(theta))+50
		#angle +=5

		if (rospy.get_param("all_robot_localised")):

			print ("robot_xy",robot_xy)
			dist_mat = distance.cdist(corner_centre, robot_xy, 'euclidean')
			row_ind, col_ind = linear_sum_assignment(dist_mat)
			
			## GOAL TRANSMISSION
			for i in range(number_of_robots):
				p = Point32()
				goal_index = row_ind[np.where(col_ind==i)][0]
				p.x = goal_xy[goal_index][0]
				p.y = goal_xy[goal_index][1]
				goal_publishers["pub"+str(i+1)].publish(p)
				#print ("GOAL | ",(x_g,y_g))
		
		rate.sleep()	

if __name__== '__main__':

	rospy.init_node("swarm",anonymous = True)
	try:
		init = (rospy.get_param("init"))
		connections = [init[a] for a in init]
		number_of_robots = len(connections)
		robot_xy = [[0.0,0.0]]*number_of_robots
		goal_xy = corner_centre
		talker()
	except rospy.ROSInterruptException:
		pass