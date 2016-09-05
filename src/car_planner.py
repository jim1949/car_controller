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
ranges_num=37
class Car_data():
    def __init__(self):
        self.car_init_x=-1.5
        self.car_init_y=-15.0
        self.scanarea=14.0
        self.car_init_yaw=-pi/2
        self.path=np.array([[-1.5,40.0],[1.1,40.0],[1.1,-40.0],[-1.5,-40.0]])

        self.distance_to_left_pavement=5.0
        self.distance_to_right_pavement=6.0
        #based on vehicle and pedestrians
        self.bufdistance=2.0
        self.final_distance=8.0

        self.length=1.5
        self.width=1.5

class Pioneer_data():
    def __init__(self):
        self.pioneer_init_x=0.0
        self.pioneer_init_y=0.0
        self.scanarea=8.0
        self.pioneer_init_yaw=-pi/2
        self.path=np.array([[-2.0,0.0],[8.0,0.0]])
        self.distance_to_left_pavement=4.0
        self.distance_to_right_pavement=6.0
        self.bufdistance=0.5
        self.final_distance=8.0

        self.length=0.5
        self.width=0.5
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

        self.init_x=0.0
        self.init_y=0.0
        self.init_yaw=0.0


        self.init_setvalue_yaw=0.0
        self.startflag=True
        self.destflag=False
        self.stopflag=False#flag if detect any pedestrains
        self.init_setflag=False

        self.nearest_reading=float('Inf')
        self.w=0.0
        self.v=0.0
        self.corner_num=0

        self.out_v=0.0
        self.out_w=0.0

        self.distancelength=0.0

        self.waitmessage=0

    
        self.bufdistance=0.0

        self.length=0
        self.width=0

        self.distance_to_left_pavement=0.0
        self.distance_to_left_pavement=0.0

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
        self.x=int(input("0:simulation only,1:simulation for pioneer with GPS sensor,2:simulation for pioneer with odometry sensor,3:real world for pioneer\n"))

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
            self.people_file=open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/people_position_simulation.dat","a")

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
        self.start_judgedistance=self.distance_to_left_pavement-self.max_v*self.max_v/(2*self.max_a)-self.bufdistance

        # input("are you sure?")
    def setcardata(self,pioneerdata,cardata):
        self.max_v=1.0
        self.max_a=4.0
        self.max_jerk=4.0

        self.nearestreading_stop=2.0
        
        self.init_setflag=True
        #car information
        self.path=cardata.path
        self.scanarea=cardata.scanarea
        self.init_x=cardata.car_init_x
        self.init_y=cardata.car_init_y
        self.init_yaw=cardata.car_init_yaw
        self.distance_to_left_pavement=cardata.distance_to_left_pavement
        self.distance_to_right_pavement=cardata.distance_to_right_pavement
        self.bufdistance=cardata.bufdistance
        self.length=cardata.length
        self.width=cardata.width
        self.final_distance=cardata.final_distance
        print("choose simulation,max_v:%f,my acceleration:%f"%(self.max_v,self.max_a))
    def setpioneer_GPS_data(self,pioneerdata,cardata):
        self.max_v=1.0
        self.max_a=4.0
        self.max_jerk=1.0 

        self.nearestreading_stop=1.0

        self.init_setflag=True
        #pioneer information
        self.path=pioneerdata.path
        self.scanarea=pioneerdata.scanarea
        self.init_x=pioneerdata.pioneer_init_x
        self.init_y=pioneerdata.pioneer_init_y
        self.init_yaw=pioneerdata.pioneer_init_yaw
        self.distance_to_left_pavement=pioneerdata.distance_to_left_pavement
        self.distance_to_right_pavement=pioneerdata.distance_to_right_pavement
        self.bufdistance=pioneerdata.bufdistance
        self.length=pioneerdata.length
        self.width=pioneerdat.width
        self.final_distance=pioneerdata.final_distance
    def setpioneer_odom_data(self,pioneerdata,cardata):
        self.max_v=1.0
        self.max_a=0.5
        self.max_jerk=1.0 

        self.nearestreading_stop=1.0
        self.scanarea=pioneerdata.scanarea

        self.init_setflag=False
        #pioneer information
        self.path=pioneerdata.path

        self.init_x=pioneerdata.pioneer_init_x
        self.init_y=pioneerdata.pioneer_init_y
        self.init_yaw=pioneerdata.pioneer_init_yaw

        self.distance_to_left_pavement=pioneerdata.distance_to_left_pavement
        self.distance_to_right_pavement=pioneerdata.distance_to_right_pavement
        self.bufdistance=pioneerdata.bufdistance
        self.length=pioneerdata.length
        self.width=pioneerdata.width
        self.final_distance=pioneerdata.final_distance

    def setdefault(self,position,yaw):
        if self.init_setflag==False and self.waitmessage>30:
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
    def v_controller(self,peoplestates):
        if self.stage==1:
            self.acceleration()
            if self.v==self.max_v:
                self.stage=2


        if self.stage==2:
        #the most import part in the whole decison making program
            clearflag=self.checking_pedestrians_clear(peoplestates)
            if clearflag==True:
                # self.acceleration()
                pass
            else:
                self.deacceleration()
        if self.stage==3:
            
            endflag=self.checking_destination_ditance()
            if endflag==True:
                self.deacceleration()

    def judge_stage(self):
        # if self.distancelength<self.distance_to_left_pavement-self.bufdistance:
        #     self.stage=1


        if self.distancelength>self.distance_to_right_pavement:
            self.stage=3

        else:
            self.stage=2
    ###########You get to the destination!########################")
    #
    #used only in stage 1. And need to consider about the comfort later on.
    def acceleration(self):
        self.a=self.max_a
        self.v=self.v+self.a*self.dt
        
        if self.v>self.max_v:
            self.v=self.max_v
        else:
            rospy.loginfo("Accelerate! Don't worry!")

#basic deacceleration model
    def deacceleration(self):
#v^2=2*a*s, here we stop at the bufdistance place, in the future, we may consider to slow down at the left_pavement place.
        a=self.v*self.v/(2*(self.distance_to_left_pavement-self.bufdistance))

        self.v=self.v-self.a*self.dt
        if self.v<0.0:
            self.v=0.0
# for stage 3--checking destination.
    def checking_destination_ditance(self):
        #could set the acceleration later:
        endflag=False
        if self.final_distance-self.bufdistance-self.distancelength<self.v*self.v/(2*self.max_a):
            endflag=True

        return endflag

    def checking_pedestrians_clear(self,peoplestates):
        #we can change the v_pedestrian_max to predict velocity here.
        clearflag=False
        t_pedestrian_min=(peoplestates.world_nearest_position_x-self.width/2)/v_pedestrian_max
        # rospy.loginfo("Nearest pedestrian:%f"%peoplestates.world_nearest_position_x)
        if self.v*t_pedestrian_min>self.distance_to_right_pavement-self.distancelength+self.length:
            clearflag=True
            rospy.loginfo("pedestrians are all gone!!!")

        return clearflag

    def callback_pose(self,msg):#position
        #=======================================#

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        roll,pitch,yaw = self.eulerfromquaterion(orientation.x,orientation.y,orientation.z,orientation.w)
        self.setdefault(position,yaw)
        self.car_x = (position.x-self.init_x)*cos(self.init_yaw)-(position.y-self.init_y)*sin(self.init_yaw)
        self.car_y = -(position.x-self.init_x)*sin(self.init_yaw)+(position.y-self.init_y)*cos(self.init_yaw)
        self.car_theta = yaw-self.init_yaw


        self.out_v=self.v
        self.out_w=self.w

    def callback_velocity(self,msg):#velocity test
        linear=msg.twist.twist.linear
        angular=msg.twist.twist.angular

        self.car_ctrlSpeed=math.sqrt(linear.x**2+linear.y**2+linear.z**2)
        self.car_ctrlSpeed_x=linear.x
        self.car_ctrlSteer=angular.z

    def callback_sick(self,msg):#scan and collision avoidance.
        self.ranges=msg.ranges
        self.stopflag=False
        self.readings=self.ranges


        for i in self.ranges[len(self.ranges)/2:]:
            if i<5:
                self.stopflag=True

        self.nearest_reading = min(self.ranges)
    def callback_people_pose(self,msg):
        self.people_x=msg.pose.position.x
        self.people_y=msg.pose.position.y

        
        # print("nearest reading: %2.2f" %nearest_reading)
    def write_msg(self,peoplestates):
        if self.init_setflag==True:
            
            if self.readings:
                self.results_file_handle.write("%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f\n" %(rospy.get_time(), self.car_x, self.car_y, self.car_theta, self.car_ctrlSpeed, self.car_ctrlSteer, peoplestates.world_nearest_position_x,peoplestates.world_nearest_position_y))
                self.people_file.write("%2.4f %2.4f %2.4f %2.4f\n"%(peoplestates.world_nearest_position_x,peoplestates.world_nearest_position_y,self.people_x,self.people_y))

                self.sick_readings.write("%2.4f  "%rospy.get_time())
                for value in self.readings:
                    self.sick_readings.write("%2.4f  " %value)              
                self.sick_readings.write("\n")
                for i in range(0,ranges_num):
                    self.people_position.write("%2.4f "%peoplestates.world_position_x[i])

                self.people_position.write("%2.4f \n"%self.people_x)
                for i in range(0,ranges_num):
                    self.people_position.write("%2.4f "%peoplestates.world_position_y[i])
                self.people_position.write("%2.4f \n"%self.people_y)
#initial particle position area
def initStateVectors(rec,sampleSize):
    sampleSize=int(sampleSize)
    xs = [random.uniform(rec[0],rec[1]) for i in range(sampleSize)]
    ys = [random.uniform(rec[2],rec[3]) for i in range(sampleSize)]
    vxs = [random.uniform(0,1) for i in range(sampleSize)]
    vys = [random.uniform(0,1) for i in range(sampleSize)]

    return([list(s) for s in zip(xs,ys,vxs,vys)])


class detect_area:
    def __init__(self,x_t,x_d,y_l,y_r):
        self.rec=[x_t,x_d,y_l,y_r]
            
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

        self.t=0

        self.v_x=0

    def update(self,path_plan):
        self.current_states(path_plan)
        self.estimate_states()

    def current_states(self,path_plan):
        self.states_measurement(path_plan)
        
    def estimate_states(self):#particle filter
        pass


    def states_measurement(self,path_plan):
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
            # if (self.world_position_y[i]>path_plan.distance_to_left_pavement) and (self.world_position_y[i]<path_plan.distance_to_right_pavement):

            if path_plan.readings[i]<path_plan.scanarea:
                sum_x=sum_x+self.world_position_x[i]
                sum_y=sum_y+self.world_position_y[i]
                self.point_no=self.point_no+1
                if self.world_nearest_position_x>abs(self.world_position_x[i]):
                    self.world_nearest_position_x=self.world_position_x[i]
                    self.world_nearest_position_y=self.world_position_y[i]
            else:
                self.world_position_x[i]=float('inf')
                self.world_position_y[i]=float('inf')
        if self.point_no!=0:
            self.world_position_x_mean=sum_x/self.point_no
            self.world_position_y_mean=sum_y/self.point_no

        rospy.loginfo("world_nearest_x:%f,y:%f"%(self.world_nearest_position_x,self.world_nearest_position_y))
        # rospy.loginfo("world_position_x:%f,y:%f"%(self.world_position_x,self.world_position_y))
        print(self.world_position_x)
        print(self.world_position_y)
        

def start():
    rospy.init_node("car_controller")
    print("Beginning....")
    cardata=Car_data()

    pioneerdata=Pioneer_data()
    peoplestates=People_states()
    path_plan=path_planner()

    path_plan.set_vehicle_data(pioneerdata,cardata)
    motion = Twist()

    #particle initialization.
    detectarea=detect_area(-10.0,10.0,-.3,-7.0)
    initialdata=initial_data(detectarea)
    measure=measurement()
    x_dsampleSize=100
    Particle_info=particle(initStateVectors(detectarea.rec,x_dsampleSize),x_dsampleSize)
    
    r=rospy.Rate(path_plan.freq)#60hz


    while not rospy.is_shutdown():
        #people mean position at current location.
        peoplestates.states_measurement(path_plan)
        #2).calculate the weights by using gauss 
        #input:
        # particles positions,(np.array(particle.svs)[:,0],np.array(particle.svs)[:,1]) 
        # measurement positions,()
        #output:guass possibility for each of them. next time how many paricles will evolve from the original one.

        
        

        #wait messages
        if path_plan.waitmessage<31:
            path_plan.waitmessage=path_plan.waitmessage+1
        else:
            path_plan.write_msg(peoplestates)
            peoplestates.update(path_plan)
        path_plan.distancelength=path_plan.distancelength+path_plan.car_ctrlSpeed*path_plan.dt
        # rospy.loginfo("distance_length:%f,  car_x:%f,  car_y:%f"%(path_plan.distancelength,path_plan.car_x,path_plan.car_y))


        stage=path_plan.judge_stage()
        path_plan.v_controller(peoplestates)


        # if path_plan.nearest_reading<1.0:
        #     path_plan.out_v=0.0
        motion.linear.x=path_plan.out_v
        # motion.linear.x=1.0

        # rospy.loginfo("speed:  %f"%path_plan.out_v)
        if path_plan.waitmessage>=30:
            # path_plan.cmd.publish(motion)
            pass
        else:
            rospy.loginfo("Don't move!")
        # rospy.loginfo("In the loop,my speed_x:%f,my angular velocity:%fmy orientation:%f"%(path_plan.car_ctrlSpeed_x,path_plan.car_ctrlSteer,path_plan.car_theta))
  
        r.sleep()

    rospy.spin()

if __name__ == '__main__':
    start()#     listener()