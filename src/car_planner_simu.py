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
from particle_filter_function import *

# from function.waypoint_following import waypoint_follower
# from function.point2point import point2point
# from function.point2point2 import point2point
# from tclass.point_rectangle import *

global v_pedestrian_max
v_pedestrian_max=1.0
global ranges_num
#need change sometimes
ranges_num=180
class Car_data():
    def __init__(self):
        self.car_init_x=5.0
        self.car_init_y=1.0
        self.scanarea=14.0
        self.car_init_yaw=-pi/2
   

        self.left_pavement_pose=-6.0/2
        self.right_pavement_pose=-9.0/2
        #based on vehicle and pedestrians
        self.bufdistance=2.0
        self.final_pose=-8.0

        self.length=2.0
        self.width=1.5

class Pioneer_data():
    def __init__(self):
        self.pioneer_init_x=0.0
        self.pioneer_init_y=0.0
        self.scanarea=8.0
        self.pioneer_init_yaw=-pi/2


        self.bufdistance =0.5
        self.final_pose =-15.0/2

        self.length=1.0
        self.width=1.0

        self.left_pavement_pose=-6.0
        self.right_pavement_pose=-9.0
        #based on vehicle and pedestrians

class Point:
    
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    
    def distance_to(self, p):
        """Calculate the distance between two points."""
        return (self - p).length()
    

class path_planner():
    def __init__(self):
        self.readings=float('Inf')*np.ones(ranges_num)
        self.sick_readings=float('Inf')*np.ones(ranges_num)
        self.car_x=0.0
        self.car_y=0.0
        self.car_theta=0.0
        self.car_ctrlSpeed=0.0
        self.car_ctrlSpeed_x=0.0
        self.car_ctrlSteer=0.0
        self.freq=6
        self.dt=1.0/self.freq

        self.a=0.2
        self.last_a=0.0
        self.jerk=0.0

        self.init_x=0.0
        self.init_y=0.0
        self.init_yaw=0.0


        self.init_setvalue_yaw=0.0
        self.startflag=True
        self.destflag=False
        # self.stopflag=False#flag if detect any pedestrains
        self.init_setflag=False

        self.nearest_reading=float('Inf')
        self.w=0.0
        self.v=0.0
        self.corner_num=0


        self.distancelength=0.0

        self.waitmessage=0.0

    
        self.bufdistance=0.0

        self.length=0.0

        self.width=0.0

        self.left_pavement_pose=0.0
        self.right_pavement_pose=0.0

        self.stage=1

        self.ranges=[]

        #people
        self.people_x=float('Inf')
        self.people_y=float('Inf')

        # 2(1.5,20)->    3(1.5,-20)
        #     ^    |
        #     |    v

        # 1(-1.5,20)<-    4(-1.5)
        self.people_subscriber=rospy.Subscriber("/people/pose",PoseStamped,self.callback_people_pose)
        # self.x=int(input("0:simulation only,1:simulation for pioneer with GPS sensor,2:simulation for pioneer with odometry sensor,3:real world for pioneer\n"))
        self.bufferarea=open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/bufferarea.dat","a")
        self.x=2
        print(self.x)

        self.people_position=open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/people_position.dat","a")
        if self.x==0:
            self.cmd = rospy.Publisher("/robot/motion", Twist, queue_size = 10)
            self.subscriber1=rospy.Subscriber("/robot/pose", PoseStamped, self.callback_pose)
            self.subscriber2=rospy.Subscriber("/robot/velocity", TwistStamped, self.callback_velocity)
            self.subscriber3=rospy.Subscriber("/robot/sick",LaserScan,self.callback_sick)
            print("choose simulation,max_v")
            self.sick_readings = open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pod_simulation_sick_data.dat", "a")
 
            self.results_file_handle = open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pod_simulation_cardata.dat","a")
            self.people_file=open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/people_position_simulation.dat","a")

        elif self.x==1:
            self.cmd = rospy.Publisher("/robot/motion", Twist, queue_size = 10)
            self.subscriber1=rospy.Subscriber("/robot/pose", PoseStamped, self.callback_pose)
            self.subscriber2=rospy.Subscriber("/robot/velocity", TwistStamped, self.callback_velocity)
            self.subscriber3=rospy.Subscriber("/robot/sick",LaserScan,self.callback_sick)
            print("choose simulation with pioneer with GPS sensor")
            self.sick_readings = open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pioneer_simulation_GPS_sick_data.dat", "a")
            self.results_file_handle = open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pioneer_simulation_GPS_cardata.dat","a")
            self.people_file=open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/people_position_simulation.dat","a")
        elif self.x==2:
            self.cmd = rospy.Publisher("/robot/motion", Twist, queue_size = 10)
            self.subscriber1=rospy.Subscriber("/robot/odom", Odometry, self.callback_pose)
            self.subscriber2=rospy.Subscriber("/robot/odom", Odometry, self.callback_velocity)
            self.subscriber3=rospy.Subscriber("/robot/sick",LaserScan,self.callback_sick)
            print("choose simulation with pioneer with odometry sensor")
            self.sick_readings = open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pioneer_simulation_sick_data.dat", "a")
            self.results_file_handle = open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pioneer_simulation_cardata.dat","a")
            self.people_file=open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/people_position_simulation.dat","a")

        else:
            self.cmd = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
            self.subscriber1=rospy.Subscriber("/pose", Odometry, self.callback_pose)
            self.subscriber2=rospy.Subscriber("/pose", Odometry, self.callback_velocity)
            self.subscriber3=rospy.Subscriber("/scan",LaserScan,self.callback_sick)
            print("choose real world with pioneer")
            self.sick_readings = open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pioneer_realworld_sick_data.dat", "a")
            self.results_file_handle = open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pioneer_realworld_cardata.dat","a")
            self.people_file=open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/people_position_realworld.dat","a")

        print(self.x)
    #1. set data

    def set_vehicle_data(self, pioneerdata,cardata):
        if self.x==0:
            self.setcardata(pioneerdata,cardata)
            
        elif self.x==1:
            self.setpioneer_GPS_data(pioneerdata,cardata)
            
            print("choose simulation with pioneer in GPS,max_v:%f,my acceleration:%f,pioneer init_x%f:,pioneer init_y%f:,"%(self.max_v,self.max_a,self.init_x,self.init_y))
        elif self.x==2:
            self.setpioneer_odom_data(pioneerdata,cardata)
           
            print("choose simulation with pioneer,max_v:%f,my acceleration:%f,pioneer init_x%f:,pioneer init_y%f:,"%(self.max_v,self.max_a,self.init_x,self.init_y))

        else:
            self.setpioneer_odom_data(pioneerdata,cardata)

            print("choose real world with pioneer,max_v:%f,my acceleration:%f"%(self.max_v,self.max_a))
        # self.start_judgedistance=self.left_pavement_pose-self.max_v*self.max_v/(2*self.max_a)-self.bufdistance

        # input("are you sure?")
    def setcardata(self,pioneerdata,cardata):
        self.max_v=1.0
        self.max_a=4.0
        self.max_jerk=4.0

        self.nearestreading_stop=2.0
        
        self.init_setflag=True
        #car information

        self.scanarea=cardata.scanarea
        self.init_x=cardata.car_init_x
        self.init_y=cardata.car_init_y
        self.init_yaw=cardata.car_init_yaw
        self.left_pavement_pose=cardata.left_pavement_pose
        self.right_pavement_pose=cardata.right_pavement_pose
        self.bufdistance=cardata.bufdistance
        self.length=cardata.length
        self.width=cardata.width
        self.final_pose=cardata.final_pose
        print("choose simulation,max_v:%f,my acceleration:%f"%(self.max_v,self.max_a))
    def setpioneer_GPS_data(self,pioneerdata,cardata):
        self.max_v=1.0
        self.max_a=4.0
        self.max_jerk=1.0 

        self.nearestreading_stop=1.0

        self.init_setflag=True
        #pioneer information

        self.scanarea=pioneerdata.scanarea
        self.init_x=pioneerdata.pioneer_init_x
        self.init_y=pioneerdata.pioneer_init_y
        self.init_yaw=pioneerdata.pioneer_init_yaw
        self.left_pavement_pose=pioneerdata.left_pavement_pose
        self.right_pavement_pose=pioneerdata.right_pavement_pose
        self.bufdistance=pioneerdata.bufdistance
        self.length=pioneerdata.length
        self.width=pioneerdat.width
        self.final_pose=pioneerdata.final_pose
    def setpioneer_odom_data(self,pioneerdata,cardata):
        self.max_v=0.5/2
        self.max_a=0.2/2
        self.max_jerk=1.0 

        self.nearestreading_stop=1.0
        self.scanarea=pioneerdata.scanarea

        self.init_setflag=False
        #pioneer information


        self.init_x=pioneerdata.pioneer_init_x
        self.init_y=pioneerdata.pioneer_init_y
        self.init_yaw=pioneerdata.pioneer_init_yaw

        self.left_pavement_pose=pioneerdata.left_pavement_pose
        self.right_pavement_pose=pioneerdata.right_pavement_pose
        self.bufdistance=pioneerdata.bufdistance
        self.length=pioneerdata.length
        self.width=pioneerdata.width
        self.final_pose=pioneerdata.final_pose

    def setdefault(self,position,yaw):
        if self.init_setflag==False and self.waitmessage>=10:
            # self.init_setvalue_x=position.x-self.init_x
            # self.init_setvalue_y=position.y-self.init_y
            # self.init_setvalue_yaw=yaw-self.init_yaw
            self.init_setflag=True
            self.init_x=position.x
            self.init_y=position.y
            self.init_yaw=yaw+pi/2
            # rospy.loginfo("init_x:%f,init_y:%f,init_yaw:%f"%(self.init_x,self.init_y,self.init_yaw))
    def eulerfromquaterion(self,q0,q1,q2,q3):
        
        yaw=math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
        pitch=math.asin(2*(q0*q2-q3*q1))
        roll=math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))
        # print("steering angle")
        # #experiment shows this is the yaw angle, and I don't know why.roll and yaw are different...(have changed the name already)
        # print(yaw*180/pi)
        return roll,pitch,yaw
    #2.controller overall
  

    def callback_pose(self,msg):#position
        #=======================================#

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        # rospy.loginfo("position_x:%f,position_y:%f"%(position.x,position.y))

        roll,pitch,yaw = self.eulerfromquaterion(orientation.x,orientation.y,orientation.z,orientation.w)
        self.setdefault(position,yaw)
        self.car_x = (position.x-self.init_x)*cos(yaw-self.init_yaw)-(position.y-self.init_y)*sin(yaw-self.init_yaw)
        self.car_y = -(position.x-self.init_x)*sin(yaw-self.init_yaw)+(position.y-self.init_y)*cos(yaw-self.init_yaw)
        self.car_theta = yaw-self.init_yaw


    def callback_velocity(self,msg):#velocity test
        linear=msg.twist.twist.linear
        angular=msg.twist.twist.angular

        self.car_ctrlSpeed=math.sqrt(linear.x**2+linear.y**2+linear.z**2)
        self.car_ctrlSpeed_x=linear.x
        self.car_ctrlSteer=angular.z

    def callback_sick(self,msg):#scan and collision avoidance.
        self.ranges=msg.ranges
       
        self.readings=self.ranges


        # for i in self.ranges[len(self.ranges)/2:]:
        #     if i<5:
        #         self.stopflag=True

        self.nearest_reading = min(self.ranges)
    def callback_people_pose(self,msg):
        self.people_x=msg.pose.position.x
        self.people_y=msg.pose.position.y-8
        
        # print("nearest reading: %2.2f" %nearest_reading)
    def write_msg(self,peoplestates,motionstate,estimation):
        if self.init_setflag==True:
            
            if self.readings:

                self.results_file_handle.write("%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f\n" %(rospy.get_time(), self.car_x, self.car_y, self.car_theta, self.car_ctrlSpeed, self.car_ctrlSteer, peoplestates.world_nearest_position_x,peoplestates.world_nearest_position_y))
                self.people_file.write(" %2.4f %2.4f\n"%(self.people_x,self.people_y))

                self.sick_readings.write("%2.4f  "%rospy.get_time())
                for value in self.readings:
                    self.sick_readings.write("%2.4f  " %value)              

                for i in range(0,ranges_num):
                    self.people_position.write("%2.4f "%peoplestates.world_position_x[i])

                for i in range(0,ranges_num):
                    self.people_position.write("%2.4f "%peoplestates.world_position_y[i])
                self.sick_readings.write("\n")
                # self.people_position.write("%2.4f \n"%self.people_x)
                # for i in range(0,ranges_num):
                #     self.people_position.write("%2.4f "%peoplestates.world_position_y[i])
                # self.people_position.write("%2.4f \n"%self.people_y)
                error=math.sqrt((estimation.pose_x-peoplestates.world_position_x_mean)**2+(estimation.pose_y-peoplestates.world_position_y_mean)**2)
                self.people_position.write("%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f\n"%(rospy.get_time(), peoplestates.world_position_x_mean,peoplestates.world_position_y_mean,estimation.v_x,estimation.pose_x,estimation.pose_y,error))

                self.bufferarea.write("%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f\n "%(rospy.get_time(),estimation.pose_x,estimation.pose_y,estimation.v_x,motionstate.rec[0],motionstate.rec[1],motionstate.rec[2],motionstate.rec[3],self.car_x,self.car_y,self.car_ctrlSpeed,peoplestates.world_position_x_mean,peoplestates.world_position_y_mean,self.people_x,self.people_y,motionstate.a,motionstate.jerk))




class particle:
    def __init__(self,svs,x_dsampleSize):
        #particle states here.
        self.svs=svs
        self.x_dsampleSize=x_dsampleSize
    def likelihood(self):
        pass
    def distance_sq(self,x1,y1,x2,y2):
        distancesq=((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
        return distancesq
class Estimation():
    def __init__(self):
        self.pose_x=0.0
        self.pose_y=0.0
        self.pose_x_posterori=0.0
        self.pose_y_posterori=0.0
        self.v_x=0.0
        self.v_y=0.0
        self.v_x_max=0.0
        self.v_x_min=0.0
        self.estimate_x=0.0
        self.estimate_y=0.0

    def estimate_velocity(self,peoplestates,dt,t):
        if t==0:
            self.v_x=0.0
            self.v_y=0.0
        else:
            self.v_x=(peoplestates.world_position_x_mean-peoplestates.world_position_x_last_mean)/dt
            self.v_y=(peoplestates.world_position_x_mean-peoplestates.world_position_y_last_mean)/dt
        if self.v_x_min>self.v_x:
            self.v_x_min=self.v_x
        if self.v_x_max<self.v_x:
            self.v_x_max=self.v_x

        # rospy.loginfo("v_x:%f"%self.v_x)

class Weight():
    def __init__(self,Particle_info):
        self.update(Particle_info)
    def update(self,Particle_info):
        self.gauss=np.zeros(Particle_info.x_dsampleSize)
        self.particle_gauss_no=np.zeros(Particle_info.x_dsampleSize)
        self.sum_gauss=0.0
        self.i=0
    def gauss_possibility(self,Particle_info,peoplestates):
        sigma=0.2#need to adjust this parameter
        self.update(Particle_info)
        for i in range(0,Particle_info.x_dsampleSize):
            distancesq=Particle_info.distance_sq(Particle_info.svs[i][0],Particle_info.svs[i][1],peoplestates.world_position_x_mean,peoplestates.world_position_y_mean)

            self.gauss[i]=np.exp(-0.5*distancesq/(sigma*sigma)) / (np.sqrt(2*np.pi)*sigma)

        self.sum_gauss=sum(self.gauss)
        print("sum_gauss%f:"%self.sum_gauss)

        # # print("i:%d"%self.i)

        # print(self.gauss)
        return self.gauss

    def normalization(self,x_dsampleSize):

        for i in range(0,x_dsampleSize):
            if self.sum_gauss==0:
                self.gauss[i]=1
            else:
                self.gauss[i]=self.gauss[i]/self.sum_gauss
                self.particle_gauss_no[i]=int(x_dsampleSize*self.gauss[i])
        left_i=int(x_dsampleSize-sum(self.particle_gauss_no))
        # print(left_i)
        #usually here left_i>0,so we need to make sure it can be 0.
        #Adding some random particles to maintain the size as the same.
        left_array=[]
        #(1)store index of deleted particles in left_array.empty points...
        for i in range(0,x_dsampleSize):
            
            if self.particle_gauss_no[i]==0:
                left_array.append(i)
        # print("len:%d"%len(left_array))

        add_array=[]

        for i in range(0,left_i):
        #(2)random select the index of left_array, and save them ...random choose.
            # print("i:%d"%i)
            # print(len(left_array))
            
            x=random.randint(0,len(left_array)-1)
            add_array.append(left_array[x])
            del left_array[x]
        for i in add_array:
            self.particle_gauss_no[i]+=1

class detect_area:
    def __init__(self,x_t,x_d,y_l,y_r):
        self.rec=[x_t,x_d,y_l,y_r]

def resampling_gauss(particle,weight,x_dsampleSize,v_x,v_y,dt):

    #resampling process 1: at local area
    new_particle=[]
    for i in range(0,x_dsampleSize):
        if weight.particle_gauss_no[i]!=0:
            rec=[particle.svs[i][0]-0.3,particle.svs[i][0]+0.3,particle.svs[i][1]-0.3,particle.svs[i][1]+0.3]
            new_particle.extend(initStateVectors(rec,weight.particle_gauss_no[i]))
    particle.svs=new_particle

    # print(particle.svs)
    #resampling process 2: motion model
    new_particle=[]
    for i in range(0,x_dsampleSize):
        ##need to be careful here there is no dt.
        # print("i%d"%i)
        x1=particle.svs[i][0]+(v_x+random.uniform(0,0.3))*dt
        x2=particle.svs[i][0]-(v_x+random.uniform(0,0.3))*dt
        y1=particle.svs[i][1]+(v_x+random.uniform(0,0.3))*dt
        y2=particle.svs[i][1]-(v_x+random.uniform(0,0.3))*dt
        # print("x1:%fx2:%f,y1:%f,y2:%f"%(x1,x2,y1,y2))
        rec=[particle.svs[i][0]+(v_x+random.uniform(0,0.3))*dt,particle.svs[i][0]+(v_x-random.uniform(0,0.3))*dt,particle.svs[i][1]+(v_y+random.uniform(0,0.3))*dt,particle.svs[i][1]-(v_y+random.uniform(0,0.3))*dt]
        new_particle.extend(initStateVectors(rec,weight.particle_gauss_no[i]))
    particle.svs=new_particle
            
class People_states():
    def __init__(self):
        self.position_x_array=[]
        self.position_y_array=[]
        self.velocity_x_array=[]
        self.velocity_y_array=[]
        self.local_position_x=float('Inf')*np.ones(ranges_num)
        self.local_position_y=float('Inf')*np.ones(ranges_num)
        self.world_position_x=float('Inf')*np.ones(ranges_num)
        self.world_position_y=float('Inf')*np.ones(ranges_num)
        self.world_nearest_position_x=float('Inf')
        self.world_nearest_position_y=float('Inf')

        #self.world_position_x->self.pose_x
        #self.world_position_y->self.pose_y
        self.world_position_x_mean=1e3
        self.world_position_y_mean=1e3
        self.world_position_x_last_mean=1e3
        self.world_position_y_last_mean=1e3

        self.point_no=0
        self.velocity_x=0.0
        self.velocity_y=0.0

        self.estimate_x=[]
        self.estimate_y=[]

        self.possibility=[]

        self.length=1.0
        self.width=1.0

        self.t=0

        self.v_x=0

    def update(self,path_plan):
 
        self.world_position_x_last_mean=self.world_position_x_mean
        self.world_position_y_last_mean=self.world_position_y_mean


    def states_measurement(self,path_plan,detectarea):
        self.world_nearest_position_x=float('Inf')
        #help to estimate the pose_mean
        sum_x=0.0
        sum_y=0.0
        self.point_no=0
        for i in range(0,ranges_num,1):#0,1,2,3,...,ranges_num
            # if self.local_position_x[i]<0.1:
            #     self.local_position_x[i]=float('Inf')
            # if self.local_position_y[i]<0.1:
            #     self.local_position_y[i]=float('Inf')
            # rospy.loginfo("i:%f"%i)
            # rospy.loginfo("path_plan.readings%f",path_plan.readings[i])
            # rospy.loginfo("local_position_x%f",self.local_position_x[i])
            # rospy.loginfo("i:%f"%i)
            # rospy.loginfo("path_plan.readings:%f"%len(path_plan.readings))
            # rospy.loginfo("len(local_position_y):%f"%len(self.local_position_y))
            self.local_position_x[i]=path_plan.readings[i]*cos(pi*i/ranges_num)
            self.local_position_y[i]=path_plan.readings[i]*sin(pi*i/ranges_num)
            #@pay attention to that in the real world,pi-... is in simulation, and pi-... in simulation will make the robot's y in positive axis
            self.world_position_x[i]=self.local_position_x[i]*cos(path_plan.car_theta-pi/2)-self.local_position_y[i]*sin(path_plan.car_theta-pi/2)+path_plan.car_x
            self.world_position_y[i]=-self.local_position_x[i]*sin(path_plan.car_theta-pi/2)+self.local_position_y[i]*cos(path_plan.car_theta-pi/2)+path_plan.car_y
        #need to add if pedestrian not going this way...(follow x axis)

        # print(self.local_position_y)
        # print(self.world_position_x)
        # print(self.world_position_y)
        #intially set the pedestrian all work here in the pavement area.
            # if (self.world_position_y[i]>path_plan.left_pavement_pose) and (self.world_position_y[i]<path_plan.right_pavement_pose):

            if path_plan.readings[i]<path_plan.scanarea:
#           
                # rospy.loginfo("wolrd_position_x:%f,world_position_y:%f,right_pose:%f,left_pose:%f"%(self.world_position_x[i],self.world_position_y[i],path_plan.right_pavement_pose,path_plan.left_pavement_pose))
                if self.world_position_x[i]<2 and self.world_position_x[i]>-2 and self.world_position_y[i]>path_plan.right_pavement_pose and self.world_position_y[i]<path_plan.left_pavement_pose:
                    sum_x=sum_x+self.world_position_x[i]
                    sum_y=sum_y+self.world_position_y[i]
                    self.point_no=self.point_no+1
                    if self.world_nearest_position_x>abs(self.world_position_x[i]):
                        self.world_nearest_position_x=self.world_position_x[i]
                        self.world_nearest_position_y=self.world_position_y[i]

                else:
                    self.world_position_x[i]=float('inf')
                    self.world_position_y[i]=float('inf')


            else:
                self.world_position_x[i]=float('inf')
                self.world_position_y[i]=float('inf')
        if self.point_no!=0:
            self.world_position_x_mean=sum_x/self.point_no
            self.world_position_y_mean=sum_y/self.point_no

        # rospy.loginfo("world_nearest_x:%f,y:%f"%(self.world_nearest_position_x,self.world_nearest_position_y))
        # rospy.loginfo("world_position_x:%f,y:%f"%(self.world_position_x,self.world_position_y))
        # print(self.world_position_x)
        # print(self.world_position_y)
 #initial particle position area
def initStateVectors(rec,sampleSize):
    sampleSize=int(sampleSize)
    xs = [random.uniform(rec[0],rec[1]) for i in range(sampleSize)]
    ys = [random.uniform(rec[2],rec[3]) for i in range(sampleSize)]
    vxs = [random.uniform(0,1) for i in range(sampleSize)]
    vys = [random.uniform(0,1) for i in range(sampleSize)]

    return([list(s) for s in zip(xs,ys,vxs,vys)]) 
 

def motion_plan(estimation,path_plan,motionstate,peoplestates,v,dt):
    s_y_min=abs(path_plan.left_pavement_pose-path_plan.car_y)-path_plan.length/2
    s_y_max=abs(path_plan.car_y-path_plan.right_pavement_pose)+path_plan.length/2
    #1.minimum/maximum time to get to the pavement and duration
    print("speed1:%f"%path_plan.car_ctrlSpeed)
    if path_plan.car_ctrlSpeed==0.00:
        tmin_car=1000
        tmax_car=1000
    else:
        tmin_car=abs(s_y_min/path_plan.car_ctrlSpeed)
        tmax_car=abs(s_y_max/path_plan.car_ctrlSpeed)# haven't got there before buffer area.
    
   
    #2.buffer area calculation
    rospy.loginfo("car_x:%fcar_y:%f,estimation.v_x:"%(path_plan.car_x,path_plan.car_y))
    x_u=path_plan.car_x+abs(estimation.v_x*tmax_car)+peoplestates.length/2+path_plan.width/2
    x_l=path_plan.car_x-abs(estimation.v_x*tmax_car)-peoplestates.length/2-path_plan.width/2
    y_l=path_plan.left_pavement_pose
    y_r=path_plan.right_pavement_pose
    motionstate.rec=[x_u,x_l,y_l,y_r]


    # rospy.loginfo("buffer area:x_l:%f,x_u:%f,y_l:%f,y_r:%f, car_x:%f, y:%f"%(x_l,x_u,y_l,y_r,path_plan.car_x,path_plan.car_y))
    distance_to_pavement=abs(path_plan.left_pavement_pose-path_plan.car_y)
    distance_to_final_pose=abs(path_plan.final_pose-path_plan.car_y)-path_plan.car_ctrlSpeed*dt
    #3 start stage 1:
    #start from the beginning point.
    if path_plan.car_y>-path_plan.max_v**2/(2*path_plan.max_a)-0.1:
        v,a=accelerate(v,path_plan.max_v,path_plan.max_a,dt)
        rospy.loginfo("The target is moving towards the pavement! People.x:%f,y:%f,speed:%f,Car.x:%f,y:%f,speed:%f acceleration:%f\n "%(estimation.pose_x,estimation.pose_y,estimation.v_x,path_plan.car_x,path_plan.car_y,path_plan.car_ctrlSpeed,a))

    else:#path_plan.car_y<-2
        #second stage 2: Slow down?
        #4.if people is in the buffer area.
        # print((estimation.pose_x >x_l and estimation.pose_x< x_u) )
        # print((estimation.pose_y>y_r and estimation.pose_y<y_l))
        

        a=0

        if (estimation.pose_x >x_l and estimation.pose_x< x_u) and (estimation.pose_y>y_r and estimation.pose_y<y_l):
            #calculate the rest distance to the pavement.

            if motionstate.dflag==0:
                motionstate.max_v=v
                motionstate.max_s=distance_to_pavement
                rospy.loginfo("if (estimation.pose_x >x_l and estimation.pose_x< x_u) and (estimation.pose_y>y_r and estimation.pose_y<y_l)\nif motionstate.dflag==0")
            else:
                v=motionstate.max_v*distance_to_pavement/motionstate.max_s
                rospy.loginfo("if not (estimation.pose_x >x_l and estimation.pose_x< x_u) and (estimation.pose_y>y_r and estimation.pose_y<y_l)\nif motionstate.dflag!=0")
            motionstate.dflag=1

            rospy.loginfo("1.The target is in the buffer area! People.x:%f,y:%f,speed:%f,Car.x:%f,y:%f,speed:%f acceleration:%f\n "%(estimation.pose_x,estimation.pose_y,estimation.v_x,path_plan.car_x,path_plan.car_y,path_plan.car_ctrlSpeed,a))
            motionstate.stopflag=0
        #5.if people is out of the buffer area.
        else: #path_plan.car_y<-2 and out of buffer area
            motionstate.stopflag=motionstate.stopflag+1
            if motionstate.stopflag>3: #path_plan.car_y<-2 and out of buffer area 
                #Third stage 3: 
 
                rospy.loginfo("2.stage3! People.x:%f,y:%f,speed:%f,Car.x:%f,y:%f,speed:%f acceleration:%f\n "%(estimation.pose_x,estimation.pose_y,estimation.v_x,path_plan.car_x,path_plan.car_y,path_plan.car_ctrlSpeed,a))
                #no matter what, start moving
                if path_plan.car_y<path_plan.left_pavement_pose+1.0/2:#path_plan.car_y<-2 and out of buffer area and path_plan.car_y<path_plan.left_pavement_pose+1
                    #approaching the target.
                    if path_plan.car_y<path_plan.final_pose+1.0/2:#path_plan.car_y<-2 and out of buffer area and path_plan.car_y<path_plan.left_pavement_pose+1 and path_plan.car_y<path_plan.final_pose+1.5
                        #decelerate.
                        v=0.0
                        # a=v*v/(2*(distance_to_final_pose))

                        # # a=path_plan.max_a
                        # v=v-a*dt
                        # if v<0:
                        #     v=0
                        # else:
                        #     pass
                        rospy.loginfo("3.The target is going to the final end!!! Car.x:%f,y:%f,speed:%f acceleration:%f\n "%(path_plan.car_x,path_plan.car_y,path_plan.car_ctrlSpeed,a))

                    else:
                    #Accelerate.
                        rospy.loginfo("4.The target is leaving the pavement! ########################################Car.x:%f,y:%f,speed:%f acceleration:%f\n "%(path_plan.car_x,path_plan.car_y,path_plan.car_ctrlSpeed,a))
                        v,a=accelerate(v,path_plan.max_v,path_plan.max_a,dt)
                        print(v)
                else:#path_plan.car_y<-2 and out of buffer area and path_plan.car_y>=path_plan.left_pavement_pose+1
                    v,a=accelerate(v,path_plan.max_v,path_plan.max_a,dt)
                    rospy.loginfo("5.Approaching the pavement")
            else: #path_plan.car_y<-2 and out of buffer area or not
                
                rospy.loginfo("6.struggling!")



        rospy.loginfo("7.position.x:%f,y:%f,v:%ffinal:%f"%(path_plan.car_x,path_plan.car_y,v,(path_plan.final_pose)))
    rospy.loginfo("distance to pavement:%f"%distance_to_pavement)        
    
    return v,a

def accelerate(v,max_v,max_a,dt):

    # print(max_a)
    v=max_a*dt+v
    if v>max_v+0.035:
        v=max_v
    a=max_a
    return v,a

class Motion_state():
    def __init__(self):
        self.rec=[]
        self.stopflag=0
        self.dflag=0
        self.max_s=0
        self.max_v=0

        self.a=0.0#for checking the acceleration
        self.last_a=0.0
        self.jerk=0.0

        self.last_v=0
        self.v=0



def start():
    rospy.init_node("car_controller")

    print("Beginning....")
    cardata=Car_data()

    pioneerdata=Pioneer_data()
    peoplestates=People_states()
    path_plan=path_planner()
    estimation=Estimation()
    path_plan.set_vehicle_data(pioneerdata,cardata)
    motion = Twist()
    motionstate=Motion_state()
    rospy.sleep(5.)


    #particle initialization.

    detectarea=detect_area(-3.0,3.0,-3.0,-6.0)
    # initialdata=initial_data(detectarea)
    x_dsampleSize=100
    Particle_info=particle(initStateVectors(detectarea.rec,x_dsampleSize),x_dsampleSize)
    RMES=[]

    r=rospy.Rate(path_plan.freq)#60hz
    dt=1.0/path_plan.freq
    i=0
    while not rospy.is_shutdown():
        motionstate.last_a=motionstate.a
        #wait messages
        # print("i:%d"%i)
        # i=i+1
        if path_plan.waitmessage<11:
            path_plan.waitmessage=path_plan.waitmessage+1
        else:
            path_plan.write_msg(peoplestates,motionstate,estimation)
            peoplestates.update(path_plan)
        path_plan.distancelength=path_plan.distancelength+path_plan.car_ctrlSpeed*path_plan.dt
        # rospy.loginfo("distance_length:%f,  car_x:%f,  car_y:%f"%(path_plan.distancelength,path_plan.car_x,path_plan.car_y))


        # if path_plan.nearest_reading<1.0:
        #     path_plan.out_v=0.0

        # motion.linear.x=1.0

        # rospy.loginfo("speed:  %f"%path_plan.out_v)
        if path_plan.waitmessage>=11:
                # path_plan.cmd.publish(motion)
                #people mean position at current location.
            peoplestates.states_measurement(path_plan,detectarea)
            #(get peoplestates.world_position_x_mean
            #peoplestates.world_position_y_mean)

            #2).calculate the weights by using gauss 
            #input:
            # particles positions,(np.array(particle.svs)[:,0],np.array(particle.svs)[:,1]) 
            # measurement positions,()
            #output:guass possibility for each of them. next time how many paricles will evolve from the original one.
            
            if peoplestates.point_no!=0:
                if peoplestates.t==1:
                    weight=Weight(Particle_info)
                    weight.gauss_possibility(Particle_info,peoplestates)

                    #Normalization
                    weight.normalization(x_dsampleSize)

                #3)prior estimation (current estimation)
                    estimation=Estimation()
                    for i in range(0,x_dsampleSize):
                        estimation.pose_x=Particle_info.svs[i][0]*weight.gauss[i]+estimation.pose_x
                        estimation.pose_y=Particle_info.svs[i][1]*weight.gauss[i]+estimation.pose_y
                    RMES.append(math.sqrt((estimation.pose_x-peoplestates.world_position_x_mean)**2+(estimation.pose_y-peoplestates.world_position_y_mean)**2))
                #4)Resampling(twice along with the motion)
                    estimation.estimate_velocity(peoplestates,dt,peoplestates.t)

                    resampling_gauss(Particle_info,weight,x_dsampleSize,estimation.v_x,estimation.v_y,dt)
                    peoplestates.update(path_plan)

                #5)posterior estimation
                    estimation=Estimation()
                    for i in range(0,x_dsampleSize):
                        estimation.pose_x=Particle_info.svs[i][0]+estimation.pose_x
                        estimation.pose_y=Particle_info.svs[i][1]+estimation.pose_y
                    estimation.pose_x=estimation.pose_x/x_dsampleSize
                    estimation.pose_y=estimation.pose_y/x_dsampleSize
                    print("estimation_pose_x:%f"%estimation.pose_x)
                    print("estimation_pose_y:%f"%estimation.pose_y)
                    print("peoplestate.pose_x%f"%peoplestates.world_position_x_mean)
                    print("peoplestate.pose_y%f"%peoplestates.world_position_y_mean)

                peoplestates.t=1
            else:
                estimation=Estimation()
                peoplestates.t=0
            
            path_plan.v,a=motion_plan(estimation,path_plan,motionstate,peoplestates,path_plan.v,dt)

 

        else:
            rospy.loginfo("Don't move!")
        rospy.loginfo("velocity:%f"%path_plan.v)
        #velocity check, velocity and acceleration is in the range.
        motionstate.v=path_plan.v
        motionstate.a=(motionstate.v-motionstate.last_v)/dt
        if motionstate.a>path_plan.max_a:
            motionstate.a=path_plan.max_a
        elif motionstate.a<-path_plan.max_a:
            motionstate.a=-path_plan.max_a
        motionstate.v=motionstate.last_v+motionstate.a*dt
        if motionstate.v>path_plan.max_v:
            motionstate.v=path_plan.max_v
        motion.linear.x=path_plan.v
        # motion.linear.x=0.1
        path_plan.cmd.publish(motion)
        motionstate.jerk=motionstate.a-motionstate.last_a
        



        # print(peoplestates.world_position_y)
        # print(path_plan.readings)
        # motionstate.last_v=motionstate.v
        # rospy.loginfo("So the acceleration is:%f"%motionstate.a)
        rospy.loginfo("people_x:%f,people_y:%f"%(peoplestates.world_position_x_mean,peoplestates.world_position_y_mean))
        # rospy.loginfo("In the loop,my speed_x:%f,my angular velocity:%fmy orientation:%f"%(path_plan.car_ctrlSpeed_x,path_plan.car_ctrlSteer,path_plan.car_theta))

        #motion plan
        #input: estimation.pose_x,estimation.pose_y,estimation.v_x,estimation.v_y,estimation.v_x_min/max,
        #input: path_plan.car_x,path_plan.car_y,path_plan.car_ctrlSpeed

        
        r.sleep()

    rospy.spin()

if __name__ == '__main__':
    start()#     listener()
