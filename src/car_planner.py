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
# from function.waypoint_following import waypoint_follower
# from function.point2point import point2point
# from function.point2point2 import point2point
# from tclass.point_rectangle import *

global v_pedestrian_max
v_pedestrian_max=1.0
global ranges_num
ranges_num=36
class Car_data():
    def __init__(self):
        self.car_init_x=-1.5
        self.car_init_y=-15.0
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
    
    """A point identified by (x,y) coordinates.
    
    supports: +, -, *, /, str, repr
    
    length  -- calculate length of vector to point from origin
    distance_to  -- calculate distance between two points
    as_tuple  -- construct tuple (x,y)
    clone  -- construct a duplicate
    integerize  -- convert x & y to integers
    floatize  -- convert x & y to floats
    move_to  -- reset x & y
    slide  -- move (in place) +dx, +dy, as spec'd by point
    slide_xy  -- move (in place) +dx, +dy
    rotate  -- rotate around the origin
    rotate_about  -- rotate around another point
    """
    
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y
    
    def __add__(self, p):
        """Point(x1+x2, y1+y2)"""
        return Point(self.x+p.x, self.y+p.y)
    
    def __sub__(self, p):
        """Point(x1-x2, y1-y2)"""
        return Point(self.x-p.x, self.y-p.y)
    
    def __mul__( self, scalar ):
        """Point(x1*x2, y1*y2)"""
        return Point(self.x*scalar, self.y*scalar)
    
    def __div__(self, scalar):
        """Point(x1/x2, y1/y2)"""
        return Point(self.x/scalar, self.y/scalar)
    
    def __str__(self):
        return "(%s, %s)" % (self.x, self.y)
    
    def __repr__(self):
        return "%s(%r, %r)" % (self.__class__.__name__, self.x, self.y)
    
    def length(self):
        return math.sqrt(self.x**2 + self.y**2)
    
    def distance_to(self, p):
        """Calculate the distance between two points."""
        return (self - p).length()
    
    def as_tuple(self):
        """(x, y)"""
        return (self.x, self.y)
    
    def clone(self):
        """Return a full copy of this point."""
        return Point(self.x, self.y)
    
    def integerize(self):
        """Convert co-ordinate values to integers."""
        self.x = int(self.x)
        self.y = int(self.y)
    
    def floatize(self):
        """Convert co-ordinate values to floats."""
        self.x = float(self.x)
        self.y = float(self.y)
    
    def move_to(self, x, y):
        """Reset x & y coordinates."""
        self.x = x
        self.y = y
    
    def slide(self, p):
        '''Move to new (x+dx,y+dy).
        
        Can anyone think up a better name for this function?
        slide? shift? delta? move_by?
        '''
        self.x = self.x + p.x
        self.y = self.y + p.y
    
    def slide_xy(self, dx, dy):
        '''Move to new (x+dx,y+dy).
        
        Can anyone think up a better name for this function?
        slide? shift? delta? move_by?
        '''
        self.x = self.x + dx
        self.y = self.y + dy
    
    def rotate(self, rad):
        """Rotate counter-clockwise by rad radians.
        
        Positive y goes *up,* as in traditional mathematics.
        
        Interestingly, you can use this in y-down computer graphics, if
        you just remember that it turns clockwise, rather than
        counter-clockwise.
        
        The new position is returned as a new Point.
        """
        s, c = [f(rad) for f in (sin, cos)]
        x, y = (c*self.x - s*self.y, s*self.x + c*self.y)
        return Point(x,y)
    
    def rotate_about(self, p, theta):
        """Rotate counter-clockwise around a point, by theta degrees.
        
        Positive y goes *up,* as in traditional mathematics.
        
        The new position is returned as a new Point.
        """
        result = self.clone()
        result.slide(-p.x, -p.y)
        result.rotate(theta)
        result.slide(p.x, p.y)
        return result

class path_planner():
    def __init__(self):
        self.readings=[]
        self.sick_readings=[]
        self.car_x=0.0
        self.car_y=0.0
        self.car_theta=0.0
        self.car_ctrlSpeed=0.0
        self.car_ctrlSpeed_x=0.0
        self.car_ctrlSteer=0.0
        self.freq=6
        self.dt=1.0/self.freq
        self.a=2.0

        self.init_x=0.0
        self.init_y=0.0
        self.init_yaw=0.0


        self.init_setvalue_yaw=0.0
        self.startflag=True
        self.destflag=False
        self.stopflag=False#flag if detect any pedestrains
        self.init_setflag=False

        self.nearest_reading=1e7
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

        self.stage=0

        self.ranges=[]

        # 2(1.5,20)->    3(1.5,-20)
        #     ^    |
        #     |    v

        # 1(-1.5,20)<-    4(-1.5)
        
        self.x=int(input("0:simulation only,1:simulation for pioneer with GPS sensor,2:simulation for pioneer with odometry sensor,3:real world for pioneer\n"))
        print(self.x)

        if self.x==0:
            self.cmd = rospy.Publisher("/robot/motion", Twist, queue_size = 10)
            self.subscriber1=rospy.Subscriber("/robot/pose", PoseStamped, self.callback_pose)
            self.subscriber2=rospy.Subscriber("/robot/velocity", TwistStamped, self.callback_velocity)
            self.subscriber3=rospy.Subscriber("/robot/sick",LaserScan,self.callback_sick)
            print("choose simulation,max_v")
            self.sick_readings = open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pod_simulation_sick_data.dat", "a")
 
            self.results_file_handle = open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pod_simulation_cardata.dat","a")


        elif self.x==1:
            self.cmd = rospy.Publisher("/robot/motion", Twist, queue_size = 10)
            self.subscriber1=rospy.Subscriber("/robot/pose", PoseStamped, self.callback_pose)
            self.subscriber2=rospy.Subscriber("/robot/velocity", TwistStamped, self.callback_velocity)
            self.subscriber3=rospy.Subscriber("/robot/sick",LaserScan,self.callback_sick)
            print("choose simulation with pioneer with GPS sensor")
            self.sick_readings = open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pioneer_simulation_GPS_sick_data.dat", "a")
            self.results_file_handle = open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pioneer_simulation_GPS_cardata.dat","a")
        elif self.x==2:
            self.cmd = rospy.Publisher("/robot/motion", Twist, queue_size = 10)
            self.subscriber1=rospy.Subscriber("/robot/odom", Odometry, self.callback_pose)
            self.subscriber2=rospy.Subscriber("/robot/odom", Odometry, self.callback_velocity)
            self.subscriber3=rospy.Subscriber("/robot/sick",LaserScan,self.callback_sick)
            print("choose simulation with pioneer with odometry sensor")
            self.sick_readings = open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pioneer_simulation_sick_data.dat", "a")
            self.results_file_handle = open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pioneer_simulation_cardata.dat","a")

        else:
            self.cmd = rospy.Publisher("/cmd_vel", Twist, queue_size = 10)
            self.subscriber1=rospy.Subscriber("/pose", Odometry, self.callback_pose)
            self.subscriber2=rospy.Subscriber("/pose", Odometry, self.callback_velocity)
            self.subscriber3=rospy.Subscriber("/scan",LaserScan,self.callback_sick)
            print("choose real world with pioneer")
            self.sick_readings = open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pioneer_realworld_sick_data.dat", "a")
            self.results_file_handle = open("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pioneer_realworld_cardata.dat","a")

        print(self.x)


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
        self.max_a=4.0
        self.max_jerk=1.0 

        self.nearestreading_stop=1.0

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

    def judge_stage(self):
        if self.distancelength<self.distance_to_left_pavement-self.bufdistance:
            self.stage=1

        elif self.distancelength>self.distance_to_right_pavement:
            self.stage=3

        else:
            self.stage=2




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
    def v_controller(self,peoplestates):
        if self.stage==1:
            self.acceleration()
        if self.stage==2:
            clearflag=self.checking_pedestrians_clear(peoplestates)
            if clearflag==True:
                self.acceleration()
            else:
                self.deacceleration()
        if self.stage==3:
            
            endflag=self.checking_destination_ditance()
            if endflag==True:
                self.deacceleration()

            # rospy.loginfo("################Do I really need to deaccelerate?#################################")
            # rospy.loginfo("Let's run to the end!!############")
            # rospy.loginfo("#############Yes, you need deaccelete, pedestrians are there!########################")
            # rospy.loginfo("#############You get to the destination!########################")


    def checking_destination_ditance(self):
        #could set the acceleration later:
        endflag=False
        if self.final_distance-self.bufdistance-self.distancelength<self.v*self.v/(2*self.max_a):
            endflag=True

        return endflag

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

    def checking_pedestrians_clear(self,peoplestates):
        #we can change the v_pedestrian_max to predict velocity here.
        clearflag=False
        t_pedestrian_min=(peoplestates.world_nearest_position_x-self.width/2)/v_pedestrian_max
        rospy.loginfo("Nearest pedestrian:%f"%peoplestates.world_nearest_position_x)
        if self.v*t_pedestrian_min>self.distance_to_right_pavement-self.distancelength+self.length:
            clearflag=True
            rospy.loginfo("pedestrians are all gone!!!")

        return clearflag

    def callback_pose(self,msg):#position
        #=======================================#

        fid = open('nearest_reading.dat', 'a')

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        roll,pitch,yaw = self.eulerfromquaterion(orientation.x,orientation.y,orientation.z,orientation.w)
        self.setdefault(position,yaw)
        self.car_x = (position.x-self.init_x)*cos(self.init_yaw)-(position.y-self.init_y)*sin(self.init_yaw)
        self.car_y = -(position.x-self.init_x)*sin(self.init_yaw)+(position.y-self.init_y)*cos(self.init_yaw)
        self.car_theta = yaw-self.init_yaw
        fid.write("%s\n" %self.nearest_reading)

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

        
        # print("nearest reading: %2.2f" %nearest_reading)
    def write_msg(self):
        if self.init_setflag==True:
            
            if self.readings:
                self.results_file_handle.write("%2.6f %2.6f %2.6f %2.6f %2.6f %2.6f \n" %(rospy.get_time(), self.car_x, self.car_y, self.car_theta, self.car_ctrlSpeed, self.car_ctrlSteer))
                self.sick_readings.write("%2.2f  "%rospy.get_time())
                for value in self.readings:

                    self.sick_readings.write("%2.4f  " %value)
                
                self.sick_readings.write("\n")

            
class People_states():
    def __init__(self):
        self.position_x_array=[]
        self.position_y_array=[]
        self.velocity_x_array=[]
        self.velocity_y_array=[]
        self.local_position_x=1e7*np.ones(ranges_num)
        self.local_position_y=1e7*np.ones(ranges_num)
        self.world_position_x=1e7*np.ones(ranges_num)
        self.world_position_y=1e7*np.ones(ranges_num)
        self.world_nearest_position_x=1e7
        self.world_nearest_position_y=1e7
        self.velocity_x=0
        self.velocity_y=0
        self.t=0

    def update(self,path_plan):
        self.current_states(path_plan)
        self.estimate_states()

    def current_states(self,path_plan):
        self.states_measurement(path_plan)
        
    def estimate_states(self):
        pass


    def states_measurement(self,path_plan):
        self.world_nearest_position_x=1e7

        for i in range(0,ranges_num,1):#0,1,2,3,...,ranges_num
            # if self.local_position_x[i]<0.1:
            #     self.local_position_x[i]=1e7
            # if self.local_position_y[i]<0.1:
            #     self.local_position_y[i]=1e7
            # rospy.loginfo("i:%f"%i)
            # rospy.loginfo("path_plan.readings%f",path_plan.readings[i])
            # rospy.loginfo("local_position_x%f",self.local_position_x[i])
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
            if path_plan.readings[i]<14:
                if self.world_nearest_position_x>abs(self.world_position_x[i]):
                    self.world_nearest_position_x=self.world_position_x[i]
                    self.world_nearest_position_y=self.world_position_y[i]
                #     print("this is final %f"%i)
                # rospy.loginfo("world_nearest_position_x:%f"%self.world_nearest_position_x)

        # rospy.loginfo("local_position_x num%f",len(self.local_position_x[0:ranges_num]))
        # print(path_plan.readings)
        # print(self.local_position_x)
        rospy.loginfo("world_nearest_x:%f,y:%f"%(self.world_nearest_position_x,self.world_nearest_position_y))
        
        

def start():
    rospy.init_node("car_controller")
    print("Beginning....")
    cardata=Car_data()

    pioneerdata=Pioneer_data()
    peoplestates=People_states()
    path_plan=path_planner()
    path_plan.set_vehicle_data(pioneerdata,cardata)
    motion = Twist()
    # print("got")
    # x=input("xxxx")
    r=rospy.Rate(path_plan.freq)#60hz


    while not rospy.is_shutdown():


        if path_plan.waitmessage<31:
            path_plan.waitmessage=path_plan.waitmessage+1

        else:
            path_plan.write_msg()
            peoplestates.update(path_plan)
        path_plan.distancelength=path_plan.distancelength+path_plan.car_ctrlSpeed*path_plan.dt
        # rospy.loginfo("distance_length:%f,  car_x:%f,  car_y:%f"%(path_plan.distancelength,path_plan.car_x,path_plan.car_y))


        stage=path_plan.judge_stage()
        path_plan.v_controller(peoplestates)


        # if path_plan.nearest_reading<1.0:
        #     path_plan.out_v=0.0
        motion.linear.x=path_plan.out_v


        # rospy.loginfo("speed:  %f"%path_plan.out_v)
        if path_plan.waitmessage>=30:
            pass
            
            # path_plan.cmd.publish(motion)
        else:
            rospy.loginfo("Don't move!")
        # rospy.loginfo("In the loop,my speed_x:%f,my angular velocity:%fmy orientation:%f"%(path_plan.car_ctrlSpeed_x,path_plan.car_ctrlSteer,path_plan.car_theta))
  
        r.sleep()

    rospy.spin()

if __name__ == '__main__':
    start()#     listener()