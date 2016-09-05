#!/usr/bin/env python
#using point2point rather than wayfollowing,change from car_control_original1.py version
#individual control script to control the car
#add new controller inside of it
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from math import pi,sin,cos

global v_pedestrian_max
v_pedestrian_max=1.0
global ranges_num
#need change sometimes
ranges_num=37
# class Point:
# 	def __init__(self,x=0.0,y=0.0):
# 		self.x=x
# 		self.y=y
# 	def distance_to(self,p):
# 		return (self-p).length()
class planner():
	def __init__(self):
		self.freq=60
		self.dt=1.0/self.freq
        self.x=int(input("2:simulation for pioneer with odometry sensor,3:real world for pioneer\n"))
    	print(self.x)

    	self.car_states_initial()
        if self.x==2:
    		self.simulation_init()
    	else:
    		self.realworld_init()
    	#real world and simulation:
	        #1.localization initialization: init_x,init_y,init_yaw
	    	#2.states initialization: a, max_v,max_a
	    	#3.map initialization: distance to left pavement and right. pavement area's relative location to starting point;
	    	#	bufdistance
	    	#4.car width, length, final distance
	    	#5.scan sensor: area
	def simulation_init(self):
		self.init_x=0.0
		self.init_y=0.0
		self.init_yaw=-pi/2

		self.a=1.0
		self.v=0.0
		self.max_v=1.0
		self.max_a=2.0

		self.distance_to_left_pavement=4.0
		self.distance_to_right_pavement=6.0
		self.bufdistance=2.0
		self.final_distance=8.0

		self.width=1.5
		self.length=1.5

		self.scanarea=14.0
		self.range_length=37

        print("choose simulation,max_v:%f,my acceleration:%f"%(self.max_v,self.max_a))

	def realworld_init(self):
		self.init_x=0.0
		self.init_y=0.0
		self.init_yaw=-pi/2

		self.max_v=1.0
		self.max_a=2.0
		self.a=1.0
		self.v=0.0

		self.distance_to_left_pavement=4.0
		self.distance_to_right_pavement=6.0
		self.bufdistance=0.5
		self.final_distance=8.0

		self.width=0.5
		self.length=0.3
		

		self.scanarea=8.0
		self.range_length=181

        print("choose real world,max_v:%f,my acceleration:%f"%(self.max_v,self.max_a))


        #car states
	def car_states_initial(self):
		self.car_x=0.
		self.car_y=0.0
		self.car_theta=0.0
		self.car_ctrlSpeed=0.0







class Scansensor():
	def __init__(self,plan):
		
		self.readings=float('Inf')*np.ones(self.)



def start():
	rospy.init("car_controller")
	print("Beginning...")
	plan=planner()
	scansensor=Scansensor()
	motion=Twist()
	r=rospy.Rate(initialdata.freq)




if __name__='__main__':
	start()