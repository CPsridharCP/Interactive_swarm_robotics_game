#!/usr/bin/env python3
import rospy
import time
from geometry_msgs.msg import Point32
from std_msgs.msg import String
from math import cos,sin,radians
from scipy.spatial import distance
from scipy.optimize import linear_sum_assignment
import numpy as np
import math

robot_xy = ['']
goal_xy =['']
goal_publishers = {}
color_publishers = {}
goal_reached = False
goal_error = 100.0
number_of_formations = 0
formations_names = []
formation_changed = True
in_formation = False

angle_camera = -90

angle = 0
current_formation = ""
DEMO = "NYU"

corner_centre   = [[20.0,20.0],[80.0,80.0],[20.0,80.0],[80.0,20.0],[50.0,50.0]]
line_horizontal = [[10.0,50.0],[30.0,50.0],[50.0,50.0],[70.0,50.0],[90.0,50.0]]
A               = [[50.0,75.0],[40.0,50.0],[60.0,50.0],[30.0,25.0],[70.0,25.0]]
B               = [[35.0,20.0],[35.0,50.0],[35.0,80.0],[65.0,35.0],[65.0,65.0]]
C               = [[30.0,50.0],[50.0,30.0],[50.0,70.0],[70.0,30.0],[70.0,70.0]]
D               = [[70.0,50.0],[50.0,30.0],[50.0,70.0],[30.0,30.0],[30.0,70.0]]
E               = [[40.0,50.0],[30.0,20.0],[70.0,20.0],[30.0,80.0],[70.0,80.0]]
F 				= [[30.0,20.0],[30.0,50.0],[30.0,80.0],[70.0,50.0],[70.0,80.0]]
G 				= [[70.0,80.0],[70.0,20.0],[70.0,40.0],[50.0,20.0],[40.0,50.0]]
H 				= [[35.0,30.0],[35.0,50.0],[35.0,70.0],[65.0,30.0],[65.0,50.0]]
I 				= [[50.0,10.0],[50.0,30.0],[50.0,50.0],[50.0,70.0],[50.0,90.0]]
J 				= [[30.0,40.0],[70.0,40.0],[50.0,20.0],[70.0,60.0],[70.0,80.0]]
K 				= [[30.0,30.0],[30.0,50.0],[30.0,70.0],[60.0,40.0],[60.0,60.0]]
L 				= [[50.0,20.0],[20.0,20.0],[20.0,50.0],[20.0,80.0],[70.0,20.0]]
M 				= [[20.0,20.0],[20.0,80.0],[80.0,20.0],[80.0,80.0],[50.0,60.0]]
N 				= [[40.0,70.0],[60.0,70.0],[50.0,50.0],[60.0,30.0],[40.0,30.0]]
O 				= [[25.0,60.0],[30.0,20.0],[70.0,20.0],[75.0,60.0],[50.0,80.0]]
P 				= [[40.0,20.0],[40.0,50.0],[40.0,80.0],[60.0,50.0],[60.0,80.0]]
Q               = [[20.0,65.0],[50.0,20.0],[50.0,50.0],[50.0,80.0],[70.0,35.0]]
R 				= [[35.0,20.0],[35.0,50.0],[35.0,80.0],[65.0,20.0],[65.0,65.0]]
S 				= [[35.0,10.0],[65.0,30.0],[35.0,50.0],[35.0,80.0],[65.0,90.0]]
T               = [[30.0,80.0],[50.0,30.0],[50.0,50.0],[50.0,80.0],[80.0,80.0]]
U  				= [[30.0,60.0],[70.0,60.0],[30.0,40.0],[70.0,40.0],[50.0,20.0]]
V               = [[50.0,25.0],[40.0,50.0],[60.0,50.0],[30.0,75.0],[70.0,75.0]]
W 				= [[20.0,80.0],[40.0,20.0],[50.0,40.0],[60.0,20.0],[80.0,80.0]]
X               = [[20.0,20.0],[80.0,80.0],[20.0,80.0],[80.0,20.0],[50.0,50.0]]
Y 				= [[30.0,80.0],[70.0,80.0],[50.0,60.0],[50.0,40.0],[50.0,20.0]]
Z               = [[70.0,40.0],[70.0,60.0],[50.0,50.0],[30.0,60.0],[30.0,40.0]]

formations = {"corner_centre"   : corner_centre,
			  "line_horizontal" : line_horizontal,
			  "A"               : A,
			  "B"				: B,
			  "C"               : C,
			  "D"				: D,
			  "E"				: E,
			  "F"				: F,
			  "G"				: G,
			  "H"				: H,
			  "I"				: I,
			  "J"				: J,
			  "K"				: K,
			  "L"				: L,
			  "M"				: M,
			  "N"               : N,
			  "O"				: O,
			  "P"				: P,
			  "Q"				: Q,
			  "R" 				: R,
			  "S"				: S,
			  "T"				: T,
			  "U"				: U,
			  "V"				: V,
			  "W"				: W,
			  "X"				: X,
			  "Y"				: Y,
			  "Z"				: Z}

#formations = {"hmcp"  : hmcp,
#			  "heart" : heart}

def rotate(origin, point, angle):
	ox, oy = origin
	px, py = point
	qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
	qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
	return [qx, qy]

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

def demo_updater(data):
	global DEMO, current_formation,formations
	#print ("DATA IN :",data.data, "| DEMO OLD :",DEMO)
	if data.data == DEMO:
		pass
	else:
		DEMO = data.data
		current_formation = DEMO[-1]
		goal_xy = formations[current_formation]
		#print ("DEMO UPDATED TO :",DEMO)


def talker():
	global robot_xy,angle,goal_xy,corner_centre,goal_error,number_of_formations,current_formation,formation_changed, in_formation,goal_publishers,color_publishers,angle_camera,DEMO

	rate = rospy.Rate(10)

	rospy.Subscriber("/DEMO", String, demo_updater)
	for i in range(number_of_robots):
		rospy.Subscriber("/location/robot"+str(i+1), Point32, robot_locations,i)
		goal_publishers["pub{0}".format(i+1)] = rospy.Publisher("goal/robot"+str(i+1), Point32, queue_size = 10)
		color_publishers["pub{0}".format(i+1)] = rospy.Publisher("color/robot"+str(i+1), Point32, queue_size = 10)

	## SWARM MATH GOES HERE

	while not rospy.is_shutdown():

		#print("swarm_brain | ", robot_xy)

		#rad =25
		#theta = radians(angle%360)
		#x_g = int(rad*cos(theta))+50
		#y_g = int(rad*sin(theta))+50
		#angle +=5

		if (rospy.get_param("all_robot_localised")):

			dist_mat = distance.cdist(goal_xy, robot_xy, 'euclidean')#euclidean') cityblock
			row_ind, col_ind = linear_sum_assignment(dist_mat)

			## GOAL TRANSMISSION
			goal_error = 0.0
			for i in range(number_of_robots):
				p = Point32()
				goal_index = row_ind[np.where(col_ind==i)][0]
				#g_x , g_y = rotate((50,50),(goal_xy[goal_index][0],goal_xy[goal_index][1]),math.radians(-1*angle_camera))
				p.x = int(goal_xy[goal_index][0])
				p.y = int(goal_xy[goal_index][1])
				goal_error += np.linalg.norm(np.array(robot_xy[i])-np.array(goal_xy[goal_index]))
				goal_publishers["pub"+str(i+1)].publish(p)

				c = Point32()
				if in_formation:	
					c.x = 0.0
					c.y = 200.0
					c.z = 255.0
				else:
					c.x = 255.0
					c.y = 255.0
					c.z = 0.0

				color_publishers["pub"+str(i+1)].publish(c)

				#print ("GOAL | ",(x_g,y_g))

			if goal_error < (1.5 * number_of_robots) and formation_changed :
				goal_reached = True
				formation_changed = False
				goal_reached_time = time.time()
				in_formation = True
			else:
				if formation_changed:
					goal_reached = False
					goal_reached_time = time.time()
				else:
					pass

			if (time.time() - goal_reached_time) > 4.0 and goal_reached:
				#current_formation = formations_names[(formations_names.index(current_formation)+1)%number_of_formations]
				current_formation = DEMO[(DEMO.index(current_formation)+1)%len(DEMO)]
				#goal_xy = formations[current_formation]
				goal_xy = [rotate((50,50),(x[0],x[1]),math.radians(-1*angle_camera)) for x in formations[current_formation]]
				formation_changed = True
				in_formation = False
			
			#print ("goal_reached |", goal_reached, "|",goal_error, "|",current_formation)

		rate.sleep()	

if __name__== '__main__':

	rospy.init_node("swarm",anonymous = True)
	try:
		init = (rospy.get_param("init"))
		connections = [init[a] for a in init]
		number_of_robots = len(connections)
		robot_xy = [[0.0,0.0]]*number_of_robots

		formations_names = [x for x in formations.keys()]
		number_of_formations = len(formations_names)
		
		#current_formation = formations_names[0]
		current_formation = DEMO[0]
		#goal_xy = formations[current_formation]
		goal_xy = [rotate((50,50),(x[0],x[1]),math.radians(-1*angle_camera)) for x in formations[current_formation]]

		#goal_xy = line_horizontal
		talker()
	except rospy.ROSInterruptException:
		print ("Error in swarm.py")
		pass