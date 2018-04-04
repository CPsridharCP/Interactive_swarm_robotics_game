#!/usr/bin/env python3
from sphero_sprk import Sphero
import rospy
import time
import numpy as np
import kalman 
from geometry_msgs.msg import Polygon,Point32
import sys
from scipy.stats import multivariate_normal

robot_number = str(sys.argv[1])

robot = Sphero(str(sys.argv[2]))

blobs = []
position = []
goal = []
color = [255,0,0]
heading = None
init_heading = None
dist_sum = 0.0
lights_off = False

number_of_robots = None

initial_xy = []

# For Kalman Filter
x = np.matrix([[0.0], [0.0], [0.0], [0.0]]) # initial state (location and velocity)
P = np.matrix([[0. ,0.,0.,0.],[0.,0.,0.,0.],[0.,0.,100.,0.],[0.,0.,0.,100.]]) # initial uncertainty

def guass(mu, sigma2, x):
    return 1/np.sqrt(2.*np.pi*sigma2) * np.exp(-.5*(x-mu)**2 / sigma2)

def update_blob(data):
	global blobs
	buff = []
	if len(data.points)!= 0:
		for blob in data.points:
			buff.append([blob.x,blob.y])
			blobs = buff
	else:
		blobs = []

def goal_setter(data):
	global goal
	goal = [data.x,data.y]

def color_setter(data):
	global color
	color = [int(data.x),int(data.y),int(data.z)]
	#print (color)

def localiser(position,blobs):
	buff_localiser = blobs
	dist_index = [[np.linalg.norm(np.array(position)-np.array(blob)),index] for index, blob in enumerate(buff_localiser)]
	return buff_localiser[min(dist_index)[1]]


def talker():
	global position,heading,init_heading,blobs,x,P,goal,real_head,steering,dist_sum,lights_off,color

	init = (rospy.get_param("init"))
	connections = [init[a] for a in init]
	number_of_robots = len(connections)

	pub_location = rospy.Publisher("location/robot"+robot_number, Point32, queue_size = 10)

	rate = rospy.Rate(10)

	rospy.Subscriber('/blobDetection', Polygon, update_blob)
	rospy.Subscriber("/goal/robot"+robot_number, Point32, goal_setter)
	rospy.Subscriber("/color/robot"+robot_number, Point32, color_setter)

	while not rospy.is_shutdown():

		# Bluetooth Connection
		if not rospy.get_param("init/sphero"+robot_number):
			try:
				robot.connect()
				rospy.set_param("init/sphero"+robot_number, True)
				robot.set_rgb_led(255,0,0)
				robot.roll(0,0)
			except:
				pass

		# All Robot Connection Wait
		if (rospy.get_param("init/sphero"+robot_number)) and not (rospy.get_param("connection_established")):
			robot.set_rgb_led(255,0,0)
			pass

		# Robots Initial Localisation
		if (rospy.get_param("connection_established")) and not (rospy.get_param("all_robot_localised")):

			try:
				robot.set_rgb_led(0,0,1)
				#print("Lightsoff")
				#print ("Blobs",blobs)
				if len(blobs)==0:
					lights_off = True
			except AttributeError:
				pass

			locate = (rospy.get_param("locate"))
			initial_location = [locate[a] for a in locate]

			if (not (rospy.get_param("locate/sphero"+robot_number)) and ((int(robot_number) == 1) or (rospy.get_param("locate/sphero"+str((int(robot_number)-1)%len(initial_location))) == 1))) and lights_off:
				robot.set_rgb_led(255,255,255)
				time.sleep(.5)

				while not (rospy.get_param("locate/sphero"+robot_number)): 

					if len(blobs) == 1:
						position = [blobs[0][0],blobs[0][1]]
						initial_xy = position
						goal = position
						x = np.matrix([[position[0]], [position[1]], [0.0], [0.0]])
						dt_log = time.time()
						robot.set_rgb_led(0,0,1)
						rospy.set_param("locate/sphero"+robot_number, True)
						print ("Setting Robot "+ robot_number +" position to ",position)

					else:
						pass


		# Robot State Estimation and Controls
		if (rospy.get_param("all_robot_localised")):
			robot.set_rgb_led(color[0],color[1],color[2])
			#robot.roll(0,0)
			
			# Tracking
			if len(blobs) == number_of_robots:
				position = localiser(position,blobs)

			# Finds initial Heading
			if heading == None:
				robot.roll(15,0)
				#i = 0
			if init_heading==None and heading!=None:
				#i+=1
				if np.linalg.norm(np.array(position)-np.array(initial_xy)) > 5.0 :
					init_heading = heading
					#print ("Initial Heading----------------------------------------------------")
					#print(init_heading)
					#print("--------------------------------------------------------------------")
					robot.roll(0,0)
				else:
					robot.roll(15,0)

			# Kalman
			now = time.time()
			dt = now - dt_log
			dt_log = now
			x,P = kalman.filter(x,P,dt,position)
			position_old = position
			position = [np.asscalar(x[0][0]),np.asscalar(x[1][0])]

			# Heading Estimate
			if (abs(x[2][0]) > 0.3 or abs(x[3][0]) > 0.3):
				head_buff = (np.arctan2(np.array(x[3][0]),np.array(x[2][0])) * 180.0 / np.pi)[0][0]
				#heading = head_buff if head_buff >= 0.0 else head_buff+360.0   # CCW coordinates LH
				heading = -1*head_buff if head_buff <= 0.0 else 360.0-head_buff # CW coordinates RH
				steering = heading

			# Controller
			if init_heading!= None:
				if position != goal:

					to_goal_vec = np.array(goal)-np.array(position)
					to_goal_buff = int(np.arctan2(to_goal_vec[1],to_goal_vec[0]) * 180 / np.pi)
					angle_to_goal = -1*to_goal_buff if to_goal_buff <= 0.0 else 360.0-to_goal_buff

					heading_error = 180 - abs(abs(angle_to_goal - heading) - 180)
					if angle_to_goal>heading:
						if abs(angle_to_goal-heading) == heading_error:
							pass
						else:
							heading_error = -1 * heading_error
					else:
						if abs(angle_to_goal-heading) == heading_error:
							heading_error = -1 * heading_error
						else:
							pass

					"""
					# Obstacle Avoidance
					obstacle_field_force = 0.0
					#speed_force_field = 0.0
					blob_buff = blobs
					my_blob = localiser(position,blob_buff)
					blob_buff.remove(my_blob)

					for blob in blob_buff:
						mv = multivariate_normal(blob, [[16.0, 0.0], [0.0, 16.0]])
						#smv = multivariate_normal(blob, [[20.0, 0.0], [0.0, 20.0]])
						field_now = (mv.pdf((position[0],position[1]))*150000)
						
						#if field_now > 0.0:
						#	to_obstacle_vec = np.array(blob)-np.array(position)
						#	to_obstacle_buff = int(np.arctan2(to_obstacle_vec[1],to_obstacle_vec[0]) * 180 / np.pi)
						#	angle_to_obstacle = -1*to_obstacle_buff if to_obstacle_buff <= 0.0 else 360.0-to_obstacle_buff


						obstacle_field_force += field_now
						#speed_force_field    += (smv.pdf((position[0],position[1]))*150000)
					obstacle_field_force = min(int(obstacle_field_force),90)	
					"""
					
					#robot.set_rgb_led(color[0],color[1],color[2])

					# PID
					dist = np.linalg.norm(np.array(position)-np.array(goal))
					old_dist = np.linalg.norm(np.array(position_old)-np.array(goal))
					dist_sum += dist
					
					#if obstacle_field_force <1:
					speed = int(guass(0.0,8.0,(angle_to_goal-steering+heading_error)/100.0) * (15*dist + 180*(old_dist - dist) + 0.00000001*dist_sum)) #SW1 P 15 180 || SW2 P 18 160

					#if obstacle_field_force == 0.0:
					steering = int((heading+heading_error)%360-init_heading)%360 ##### CORRECT ONE
					#else:
					#	steering = int((heading-0.5*heading_error)%360-init_heading)%360
					#steering = (steering-obstacle_field_force)%360

					robot.roll(max(min(speed,35),0),steering) #SWARM1 MIN SPEED 35|| SWARM2 MIN SPEED 40
					#print ("cte      ",cte)
					#print ("u        ",u)
					#print ("speed    ",speed)
					#print ("steering ",steering)
					#print ("obstacleF",obstacle_field_force)
					#print ("position ",position)
					#print ("heading  ",heading)
					#print ("goal     ",goal)
					#print ("init_h   ",init_heading)
					#print ("")

			p = Point32()
			p.x = position[0]
			p.y = position[1]
			#p.z = float(heading)
			pub_location.publish(p)

		else:
			pass
			
		rate.sleep()

if __name__== '__main__':

	rospy.init_node("sphero"+robot_number,anonymous = True)
	try:
		talker()
	except rospy.ROSInterruptException:
		pass