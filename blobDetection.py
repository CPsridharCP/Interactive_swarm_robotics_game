#!/usr/bin/env python
from collections import deque
import numpy as np
import cv2
import imutils
import rospy
from geometry_msgs.msg import Polygon,Point32

greenLower = (100,0, 0)
greenUpper = (255,255,255)

def map_to_pix(x,y):
	x = x*7
	y = (50 + (50-y))*7
	return x,y

def map_to_xy(x,y):
	x = int(x//7)
	y = 50 + (50-(int(y//7)))
	return x,y


def talker():
	pub = rospy.Publisher('blobDetection', Polygon, queue_size = 10)
	rospy.init_node('blobDetection',anonymous = True)
	rate = rospy.Rate(10)
 

	camera = cv2.VideoCapture(1)

	while not rospy.is_shutdown():
		(grabbed, frame) = camera.read()

		frame = imutils.resize(frame, width=1200)
		height = np.size(frame,0)
		width = np.size(frame,1)
		frame = frame[int(1.3*height/10):int(9.1*height/10), int(1.75*width/10):int(7.6*width/10)]
		
		blurred = cv2.GaussianBlur(frame, (15, 15), 0)
		mask = cv2.inRange(blurred, greenLower, greenUpper)
		#kernel = np.ones((3,3),np.uint8) # was 15,15 
		#mask = cv2.erode(mask, kernel, iterations=2)
		#cv2.imshow("test",mask)

		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]

		center = None

		blobs =[]
		if len(cnts) > 0:
			for c in cnts:
				#c = max(cnts, key=cv2.contourArea)
				((x, y), radius) = cv2.minEnclosingCircle(c)
				M = cv2.moments(c)
				try:
					center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
				except:
					continue
				if radius > 3:
					cv2.circle(frame, (int(x), int(y)), int(radius),(0, 255, 255), 2)
					cv2.circle(frame, center, 5, (0, 0, 255), -1)
					buff = map_to_xy(x,y)
					blobs.append([int(buff[0]),int(buff[1])])

		p = Polygon()
		p.points = ['']*len(blobs)
		for i,blob in enumerate(blobs):
			p.points[i] = Point32(x=blob[0] , y=blob[1])

		pub.publish(p)

		print ("Blobs:",blobs)
		rate.sleep()

	#robot_positions = outbuff
		cv2.imshow("test",frame)
		key = cv2.waitKey(1) & 0xFF

if __name__== '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass