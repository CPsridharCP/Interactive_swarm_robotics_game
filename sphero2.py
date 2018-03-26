#!/usr/bin/env python3
from sphero_sprk import Sphero
import rospy
import time
import kalman
import numpy as np
from geometry_msgs.msg import Polygon,Point32

robot_number = "2"

robot = Sphero("DA:06:00:55:3A:8D")

blobs = []
position = []
goal = []
heading = None
init_heading = None
dist_sum = 0.0

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
	for blob in data.points:
		buff.append([blob.x,blob.y])
		blobs = buff

def goal_setter(data):
	global goal
	goal = [data.x,data.y]

def localiser(position,blobs):
	buff_localiser = blobs
	dist_index = [[np.linalg.norm(np.array(position)-np.array(blob)),index] for index, blob in enumerate(buff_localiser)]
	return buff_localiser[min(dist_index)[1]]


def talker():
	global position,heading,init_heading,blobs,x,P,goal,real_head,steering,dist_sum

	init = (rospy.get_param("init"))
	connections = [init[a] for a in init]
	number_of_robots = len(connections)

	pub_location = rospy.Publisher("location/robot"+robot_number, Point32, queue_size = 10)

	rate = rospy.Rate(10)

	rospy.Subscriber('/blobDetection', Polygon, update_blob)
	rospy.Subscriber("/goal/robot"+robot_number, Point32, goal_setter)

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
			pass

		# Robots Initial Localisation
		if (rospy.get_param("connection_established")) and not (rospy.get_param("all_robot_localised")):

			robot.set_rgb_led(0,0,0)
			locate = (rospy.get_param("locate"))
			initial_location = [locate[a] for a in locate]

			if not (rospy.get_param("locate/sphero"+robot_number)) and ((int(robot_number) == 1) or (rospy.get_param("locate/sphero"+str((int(robot_number)-1)%len(initial_location))) == 1)):
				robot.set_rgb_led(255,255,255)
				time.sleep(.5)

				while not (rospy.get_param("locate/sphero"+robot_number)): 

					if len(blobs) == 1:
						position = [blobs[0][0],blobs[0][1]]
						initial_xy = position
						x = np.matrix([[position[0]], [position[1]], [0.0], [0.0]])
						dt_log = time.time()
						robot.set_rgb_led(0,0,0)
						rospy.set_param("locate/sphero"+robot_number, True)
						print ("Setting Robot "+ robot_number +" position to ",position)

					else:
						pass

		# Robot State Estimation and Controls
		if (rospy.get_param("all_robot_localised")):
			robot.set_rgb_led(0,255,0)
			robot.roll(0,0)
			
			# Tracking
			if len(blobs) == number_of_robots:
				position = localiser(position,blobs)

			# Finds initial Heading
			if heading == None:
				robot.roll(10,0)
				i = 0
			if init_heading==None and heading!=None:
				i+=1
				if i==30:
					init_heading = heading
					print ("Initial Heading----------------------------------------------------")
					print(init_heading)
					print("--------------------------------------------------------------------")
					robot.roll(0,0)

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
 
					dist = np.linalg.norm(np.array(position)-np.array(goal))
					old_dist = np.linalg.norm(np.array(position_old)-np.array(goal))
					dist_sum += dist
					speed = int(guass(0.0,9.0,(angle_to_goal-steering+heading_error)/100.0) * (15*dist + 190*(old_dist - dist) + 0.00000001*dist_sum))
					steering = int((heading+heading_error)%360-init_heading)%360
					robot.roll(max(min(speed,100),0),steering)
					#print ("cte      ",cte)
					#print ("u        ",u)
					#print ("speed    ",speed)
					#print ("steering ",steering)
					print ("position ",position)
					print ("heading  ",heading)
					print ("goal     ",goal)
					print ("init_h   ",init_heading)
					print ("")

			p = Point32()
			p.x = position[0]
			p.y = position[1]
			#p.z = float(heading)
			pub_location.publish(p)

		else:
			robot.set_rgb_led(0,0,0)
			pass
			
		rate.sleep()

if __name__== '__main__':

	rospy.init_node("sphero"+robot_number,anonymous = True)
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
