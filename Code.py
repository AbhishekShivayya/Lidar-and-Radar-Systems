#! /usr/bin/env python

import rospy
from os import system
from sensor_msgs.msg import LaserScan
from math import degrees,cos,sin,sqrt
import time
import pdb
import numpy

flag1 =0
flag2 =0
i_side = 0.0

def closest_dist(msg,dummy_ranges):
	closest_distance = min(dummy_ranges)
	return closest_distance

def min_ray(closest_distance,msg,dummy_ranges):
	for i in range(0, 360):
		if closest_distance == dummy_ranges[i]:
			ray_minimum = i
			break
	return ray_minimum

def angle_calculator(ray,msg):
	angle = degrees(ray * msg.angle_increment)
	angle0 = (ray * msg.angle_increment)
	return angle0

def first_edge(minimum_ray,msg,dummy_ranges):
	flag1 =0
	inf = float('inf')
	edge1_ray =0
	edge1_dist = 0.0
	j = minimum_ray
	while(j < 360):
		if (dummy_ranges[(j + 1) % 360] == inf) and (dummy_ranges[(j + 2) % 360] != inf):
			diff = abs(dummy_ranges[j] - dummy_ranges[(j + 2) % 360])
			if (diff <= tolerence + 0.01):
				edge1_dist = dummy_ranges[(j + 2) % 360]
				edge1_ray = ((j + 2) % 360)
				j = j + 1
				if j == 360:
					j = -1
			else:
				break
		elif(dummy_ranges[(j + 1) % 360] == inf) and (dummy_ranges[(j + 2 )% 360] == inf):
			diff = abs(dummy_ranges[j] - dummy_ranges[(j + 3) % 360])
			if (diff <= tolerence + 0.015):
				edge1_dist = dummy_ranges[(j + 3) % 360]
				edge1_ray = ((j + 3) % 360)
				j = (j + 2) % 360
			else:
				break
		else:
			diff = abs(dummy_ranges[j] - dummy_ranges[(j + 1) % 360])
			if (diff < tolerence):
				edge1_dist = dummy_ranges[(j + 1) % 360]
				edge1_ray = (j + 1) % 360
			else:
                    		break
		flag1 = flag1 + 1
		j = (j+1) % 360
	return edge1_dist,edge1_ray,flag1

def second_edge(minimum_ray,msg,dummy_ranges):
	flag2 = 0
	edge2_ray =0
	edge2_dist = 0.0
	inf = float('inf')
	k = minimum_ray
	k = k%360
	while (k >= 0):
		if (dummy_ranges[(k - 1)%360] == inf) and (dummy_ranges[(k - 2) % 360] != inf):
			diff1 = abs(dummy_ranges[k] - dummy_ranges[(k - 2) % 360])
			if (diff1 <= tolerence + 0.01):
				edge2_dist = dummy_ranges[(k - 2) % 360]
				edge2_ray = (k - 2) % 360
				k = k - 1
				if k == 0:
					k = 360
			else:
				break

		elif (dummy_ranges[(k - 1)%360] == inf) and (dummy_ranges[(k - 2) % 360] == inf):
			diff1 = abs(dummy_ranges[k] - dummy_ranges[(k - 3) % 360])
			if (diff1 <= tolerence + 0.015):
				edge2_dist = dummy_ranges[(k - 3) % 360]
				edge2_ray = (k - 3) % 360
				k = (k - 2) % 360
			else:
				break
		else:
			diff1 = abs(dummy_ranges[k] - dummy_ranges[(k - 1)%360])
			if (diff1 <= tolerence):
				edge2_dist = dummy_ranges[(k - 1)%360]
				edge2_ray = (k - 1)%360
			else: 
				break
		flag2 = flag2 + 1		
		k = (k-1) % 360
	return edge2_dist,edge2_ray,flag2

def edge_calculator(dist,ang):
	x = (dist * cos(ang))
	y = (dist * sin(ang))
	edge = numpy.array((x, y))
	return edge

def call_back(msg):

	inf = float('inf')
	minimum_angle = 0.0
	closest_distance = 0.0
	angle = 0.0
	angle1 = 0.0
	angle2 = 0.0
	minimum_ray = 0
	edge1_ray = 0
	edge2_ray = 0
	edge1_dist = 0.0
	edge2_dist = 0.0
     	tolerence = 0.04
	itolerence = 0.015
	first_edge1 = float(0)
	first_edge2 = float(0)
	length = len(msg.ranges)
	dummy_ranges = [0] * length
	length1 = 0.0
	breadth = 0.0
	check = 0

	f = open("scan_data1.txt", "w+")
	for m in range(0, 360):
		f.write("%17.14f" % (msg.ranges[m]))
	f.close()

	for l in range(0,length):
		dummy_ranges[l] = msg.ranges[l]
	
	for l in range(0,length):
		if dummy_ranges[l] >= 0.60000000:
			dummy_ranges[l] = inf

	f = open("scan_data2.txt", "w+")
	for m in range(0, 360):
		f.write("%17.14f" % (dummy_ranges[m]))
	f.close()
	
	closest_distance = closest_dist(msg,dummy_ranges)
	minimum_ray = min_ray(closest_distance,msg,dummy_ranges)
	minimum_angle = angle_calculator(minimum_ray,msg)

	first_edge1,edge1_ray,flag1 = first_edge(minimum_ray,msg,dummy_ranges)
	angle1 = angle_calculator(edge1_ray,msg)

	second_edge1,edge2_ray,flag2 = second_edge(minimum_ray,msg,dummy_ranges)
	angle2 = angle_calculator(edge2_ray,msg)
	
	system('clear')
	print ('Closest distance : %12.9f ' % closest_distance)
	print(' ')

	edge0 = edge_calculator(closest_distance,minimum_angle)
	edge1 = edge_calculator(first_edge1,angle1)
	edge2 = edge_calculator(second_edge1,angle2)

	side1 = numpy.linalg.norm(edge1 - edge0)
	side2 = numpy.linalg.norm(edge2 - edge0)
	iside = numpy.linalg.norm(edge2 - edge1)
	iside1 = side1 + side2

	length1 = max((side1*100),(side2*100))
	breadth = min((side1*100),(side2*100))
	i_side = iside*100

	if flag1 >= 4 and flag2 >= 4:
		print("######################################################## Object: 1 ##############################################################")
		print(' ')
		print('Minimum distance : %12.9f ' % closest_distance)
		print('Angle of the object : %12.9f ' % degrees(minimum_angle))
		if (abs(iside - iside1) <= itolerence):
			print ("Iside : %12.9f") % (i_side)
			print(' ')

		else:
			print ("Length : %12.9f") % (length1)
			print ("Breadth : %12.9f") % (breadth)
			print(' ')
	else:
		print ('Not enough measurements')
		#print ('Number of object detected : 0')
		check = 1
		
	for j in range(2,10):
		#if check == 1:
			#break
		if edge2_ray < edge1_ray:
			for z in range((edge2_ray % 360),(edge1_ray % 360) + 1):
				dummy_ranges[z] = inf
		else:
			while ((edge2_ray % 360) != ((edge1_ray % 360) + 1)):
				dummy_ranges[(edge2_ray % 360)] = inf
				edge2_ray = edge2_ray + 1
		
		#f = open("scan_data3.txt", "w+")
		#for m in range(0, 360):
			#f.write("%17.14f" % (dummy_ranges[m]))
		#f.close()

		closest_distance = closest_dist(msg,dummy_ranges)
		minimum_ray = min_ray(closest_distance,msg,dummy_ranges)
		minimum_angle = angle_calculator(minimum_ray,msg)

		first_edge1,edge1_ray,flag1 = first_edge(minimum_ray,msg,dummy_ranges)
		angle1 = angle_calculator(edge1_ray,msg)
		second_edge1,edge2_ray,flag2 = second_edge(minimum_ray,msg,dummy_ranges)
		angle2 = angle_calculator(edge2_ray,msg)
	
		edge0 = edge_calculator(closest_distance,minimum_angle)
		edge1 = edge_calculator(first_edge1,angle1)
		edge2 = edge_calculator(second_edge1,angle2)

		side1 = numpy.linalg.norm(edge1 - edge0)
		side2 = numpy.linalg.norm(edge2 - edge0)
		iside = numpy.linalg.norm(edge2 - edge1)
		iside1 = side1 + side2

		length1 = max((side1*100),(side2*100))
		breadth = min((side1*100),(side2*100))
		i_side = iside*100

		if flag1 >= 4 and flag2 >= 4:
			print("###################################################### Object: %d ################################################################" % j)
			print(' ')
			print('Minimum distance : %12.9f ' % closest_distance)
			print('Angle of the object : %12.9f ' % degrees(minimum_angle))
			if (abs(iside - iside1) <= itolerence):
				print ("Iside : %12.9f") % (i_side)
				print(' ')
			
			else:
				print ("Length : %12.9f") % (length1)
				print ("Breadth : %12.9f") % (breadth)
				print(' ')
		else:
			#j = j-1
			#print('.................................................................................................................................')
			#print('                                                Number of Objects detected : %d' % j)
			#print('.................................................................................................................................')
			break
		
		#pdb.set_trace()		
		open("scan_data1.txt", "w").close()
		open("scan_data2.txt", "w").close()
		open("scan_data3.txt", "w").close()
	
if __name__ == "__main__":

	inf = float('inf')
	closest_distance = 0.0
	angle = 0.0
	angle1 = 0.0
	angle2 = 0.0
	minimum_ray = 0
     	tolerence = 0.04
	itolerence = 0.015
	edge1_ray = 0
	edge2_ray = 0
	first_edge1 = float(0)
	first_edge2 = float(0)
	rospy.init_node('task_lb') 
	dat = rospy.Subscriber('/scan',LaserScan,call_back)
	

	rospy.spin()
