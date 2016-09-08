#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from math import pi,sin,cos
# from function.waypoint_following import waypoint_follower
# from function.point2point import point2point
# from function.point2point2 import point2point
# from tclass.point_rectangle import *

# global v_pedestrian_max
# v_pedestrian_max=1.0
# global ranges_num


class people_planner():
    def __init__(self):
        print("people_planner start...")
        self.freq=60.0
        self.dt=1.0/self.freq
        self.t=0.0
        self.waittime=0.0
        self.distance=0.0
        self.stopflag=False
        self.x=0.0
        self.y=0.0
        self.i=0
        # self.status_environment=int(input("1,2?..."))
        self.status_environment=int(input("1:simulation, 2: real world\n"))
        print(self.status_environment)
        # print(self.status_environment)
        if self.status_environment==1:
            #simulation
            self.subscriber=rospy.Subscriber("people/odom",Odometry,self.callback_pose)
            self.cmd=rospy.Publisher("people/motion",Twist,queue_size=10)

        else:    
            self.subscriber=rospy.Subscriber("/pose",Odometry,self.callback_pose)
            self.cmd=rospy.Publisher("RosAria/cmd_vel",Twist,queue_size=10)

        self.stopping_point_time=[1,2,2]#random place, random position
        self.stopping_time =[1,5,10]



    def callback_pose(self,msg):
        position=msg.pose.pose.position
        self.x=position.x
        self.y=position.y
        rospy.loginfo("position.x:%f"%self.x)
        rospy.loginfo("position.y:%f"%self.y)


    def motion_controller(self):
        # self.v=0.1
        # if self.x>4.0
        #     self.v=0.0
        #     v_out=self.v

        self.t=self.t+self.dt
        v=0.0
        if self.stopflag==False:
	        if self.t>self.stopping_point_time[self.i]:
	        	
	        	self.waittime=self.waittime+self.dt
	        	if self.waittime>self.stopping_time[self.i]:
	        		self.waittime=0.0
	        		self.i=self.i+1
	        		if self.i==len(self.stopping_time):
	        			self.stopflag=True
			else:
				v=0.2
		else:
			rospy.loginfo("People rest at point %f" %self.i)
        v_out=0.1
        
        return v_out


def start():
    rospy.init_node("people_controleer")
    print("Beginning....")
    people_plan=people_planner()
    motion = Twist()
    r=rospy.Rate(people_plan.freq)

    while not rospy.is_shutdown():
        motion.linear.x=people_plan.motion_controller()
        # motion.linear.x=0.1
        people_plan.cmd.publish(motion)
        r.sleep()

    rospy.spin()

if __name__=='__main__':
    start()


