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
from math import pi
# from function.waypoint_following import waypoint_follower
# from function.point2point import point2point
# from function.point2point2 import point2point
# from tclass.point_rectangle import *
class Car_data():
    def __init__(self):
        self.car_init_x=-1.5
        self.car_init_y=-15.0
        self.car_init_yaw=-pi/2
        self.path=np.array([[-1.5,40],[1.1,40],[1.1,-40],[-1.5,-40]])
        self.pavement_point=np.array([-3.5,3.5])

class Pioneer_data():
    def __init__(self):
        self.pioneer_init_x=0.0
        self.pioneer_init_y=0.0
        self.pioneer_init_yaw=0.0
        self.path=np.array([[-1.5,5.0],[-1.5,-5.0]])
        self.pavement_point=np.array([-1.0,1.0])



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
        s, c = [f(rad) for f in (math.sin, math.cos)]
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
    def __init__(self,):
        self.readings=[]
        self.sick_readings=[]
        self.car_x=0.0
        self.car_y=0.0
        self.car_theta=0.0
        self.car_ctrlSpeed=0.0
        self.car_ctrlSteer=0.0
        self.freq=60
        self.dt=1.0/self.freq
        self.a=2.0

        self.checkingdistance=1e7

        self.init_setvalue_yaw=0.0
        self.startflag=True
        self.destflag=False
        self.stopflag=False#flag if detect any pedestrains

        self.nearest_reading=1e7
        self.w=0.0
        self.v=0.0
        self.corner_num=0

        self.out_v=0.0
        self.out_w=0.0



        # 2(1.5,20)->    3(1.5,-20)
        #     ^    |
        #     |    v

        # 1(-1.5,20)<-    4(-1.5)
        self.sick_readings = open("data/test2/sick_data.dat", "a")
        self.pedestrains_readings = open("data/test2/pedestrains.dat", "a")
        self.results_file_handle = open("data/test2/cardata.dat","a")
        self.cmd = rospy.Publisher("/robot/motion", Twist, queue_size = 10)
        self.x=int(input("0:simulation only,1:simulation for pioneer,2:real world for pioneer\n"))
        print(self.x)

        if self.x==0:

            self.subscriber1=rospy.Subscriber("/robot/pose", PoseStamped, self.callback1)
            self.subscriber2=rospy.Subscriber("/robot/velocity", TwistStamped, self.callback2)
            self.subscriber3=rospy.Subscriber("/robot/sick",LaserScan,self.callback3)
            print("choose simulation,max_v")

        elif self.x==1:
            self.subscriber1=rospy.Subscriber("/robot/pose", PoseStamped, self.callback1)
            self.subscriber2=rospy.Subscriber("/robot/velocity", TwistStamped, self.callback2)
            self.subscriber3=rospy.Subscriber("/robot/sick",LaserScan,self.callback3)
            print("choose simulation with pioneer")

        else:
            self.subscriber1=rospy.Subscriber("/pose/pose", Odometry, self.callback1)
            self.subscriber2=rospy.Subscriber("/pose/pose", Odometry, self.callback2)
            self.subscriber3=rospy.Subscriber("/scan",LaserScan,self.callback3)
            print("choose real world with pioneer")
        print(self.x)


    def set_vehicle_data(self, pioneerdata,cardata):
        if self.x==0:
            self.max_v=1.0
            self.max_a=4.0
            self.max_jerk=4.0
            self.checkingdistance=4.0
            self.nearestreading_stop=2.0
            self.init_setvalue=PoseStamped()
            self.init_setflag=True
            #car information
            self.path=cardata.path
            self.pavement_point=cardata.pavement_point
            self.init_x=cardata.car_init_x
            self.init_y=cardata.car_init_y
            self.init_yaw=cardata.car_init_yaw
            print("choose simulation,max_v:%f,my acceleration:%f"%(self.max_v,self.max_a))
        elif self.x==1:

            self.max_v=1.0
            self.max_a=2.0
            self.max_jerk=1.0 
            self.checkingdistance=3.0 
            self.nearestreading_stop=1.0

            self.init_setflag=True

            self.init_setvalue=PoseStamped()
            #pioneer information
            self.path=pioneerdata.path
            self.pavement_point=pioneerdata.pavement_point
            self.init_x=pioneerdata.pioneer_init_x
            self.init_y=pioneerdata.pioneer_init_y
            self.init_yaw=pioneerdata.pioneer_init_yaw
            print("choose simulation with pioneer,max_v:%f,my acceleration:%f"%(self.max_v,self.max_a))
        else:
            self.max_v=1.0
            self.max_a=2.0
            self.max_jerk=1.0 
            self.checkingdistance=3.0 
            self.nearestreading_stop=1.0

            self.init_setflag=False

            self.init_setvalue=Odometry()
            #pioneer information
            self.path=pioneerdata.path
            self.pavement_point=pioneerdata.pavement_point
            self.init_x=pioneerdata.pioneer_init_x
            self.init_y=pioneerdata.pioneer_init_y
            self.init_yaw=pioneerdata.pioneer_init_yaw
            print("choose real world with pioneer,max_v:%f,my acceleration:%f"%(self.max_v,self.max_a))
        input("are you sure?")

    def point2point(self,point1,point2,orientation1):
        orientation2=math.atan2(point2.y-point1.y,point2.x-point1.x)
        angle=orientation2-orientation1
        flag_ped=self.nearest_reading<self.nearestreading_stop
        #this is pavement
        flag_pavement=point1.y*point1.x/abs(point1.x)<self.checkingdistance and point1.y*point1.x/abs(point1.x)>0.0
        flag_line=(self.corner_num==0)or(self.corner_num==2)
        if (orientation2+pi)*180/pi<0.1:
            orientation2=pi


        distance=point1.distance_to(point2)
        #which stage it is?
        self.judge_stage(point1,point2)


        if distance<0.1:
            self.destflag=True
            self.startflag=True
            self.v=0.0
            self.w=0.0

        else:
            self.destflag=False
            if self.startflag==True:
                if abs(angle)*180/pi>1:
                    self.v=0.0
                    if orientation2-orientation1>pi/2:
                        self.w=2.0
                        rospy.loginfo("Time:%fStart moving!!I'm turning in the anticlockwise,my speed:%f,my angular velocity:%f"%(rospy.get_time(),self.v,self.w))
                    elif orientation2-orientation1<-pi/2:
                        self.w=-2.0
                        rospy.loginfo("Time:%fStart moving!I'm turning in the clockwise,my speed:%f,my angular velocity:%f"%(rospy.get_time(),self.v,self.w))
                    else:
                        # w=5*math.sin(orientation2-orientation1)#set the maximum angle velocity later on
                        self.w=5*(orientation2-orientation1)
                        if abs(orientation2-orientation1)>1:
                            self.w=(orientation2-orientation1)/abs(orientation2-orientation1)
                        rospy.loginfo("Time:%fStart moving!I'm turning ,my speed:%f,my angular velocity:%f"%(rospy.get_time(),self.v,self.w))



                else:
                    self.w=0.0
                    # v=8.0
                    self.startflag=False
                    #add velocity controller here.
                    self.v_controller(flag_ped,flag_pavement,flag_line,distance)
                    rospy.loginfo("Time:%fI'm moving in the straightline,my speed:%f,my angular velocity:%f"%(rospy.get_time(),self.v,self.w))
            else:
                self.w=(orientation2-orientation1)
                if abs(orientation2-orientation1)>1:
                    self.w=0.05*(orientation2-orientation1)/abs(orientation2-orientation1)
                self.v_controller(flag_ped,flag_pavement,flag_line,distance)
                rospy.loginfo("Time:%fI'm in the way ,my speed:%f,my angular velocity:%f"%(rospy.get_time(),self.v,self.w))
                # v=8.0
            rospy.loginfo("Time:%fAfter point2point,my speed:%f,my angular velocity:%f"%(rospy.get_time(),self.v,self.w))


        
    def v_controller(self,flag_ped,flag_pavement,flag_line,distance): 
        if flag_pavement==False:
            if distance<self.max_v*self.max_v/(2*self.max_a) and flag_line:
                self.deaccelerator()
                print(11111111)
            else:
                self.acceleretor()
                print(22222222)
        else:
            if flag_ped==False:
                self.acceleretor()
                print(33333333)
            else:
                if flag_line:
                    self.deaccelerator()
                    print(444444444)
        self.out_v=self.v
        self.out_w=self.w

    def judge_stage(self,point1,point2):
        pass
    def acceleretor(self):

        if self.v<self.max_v:
            self.v= self.v+self.a*self.dt
        else:
            self.v=self.max_v


    def deaccelerator(self):
        self.a=self.max_a
        if self.v>0.0:
            self.v= self.v-self.a*self.dt
        else:
            self.v=0.0

    def eulerfromquaterion(self,q0,q1,q2,q3):
        
        yaw=math.atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2))
        pitch=math.asin(2*(q0*q2-q3*q1))
        roll=math.atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))
        # print("steering angle")
        # #experiment shows this is the yaw angle, and I don't know why.roll and yaw are different...(have changed the name already)
        # print(yaw*180/pi)
        return roll,pitch,yaw
    def setdefault(self):
        if self.init_setflag==False:
            self.init_setvalue.pose.position.x=position.x-self.init_x
            self.init_setvalue.pose.position.y=position.y-self.init_y
            self.init_setvalue_yaw=yaw-self.init_yaw
            self.init_setflag=True




    def callback1(self,msg):#position
        #=======================================#
        fid = open('nearest_reading.dat', 'a')

        position = msg.pose.position
        orientation = msg.pose.orientation
        rospy.loginfo("positionx %f"%position.x)
        rospy.loginfo("positiony %f"%position.y)

        roll,pitch,yaw = self.eulerfromquaterion(orientation.x,orientation.y,orientation.z,orientation.w)
        self.setdefault()

        self.car_x = position.x-self.init_setvalue.pose.position.x
        self.car_y = position.y-self.init_setvalue.pose.position.y
        self.car_theta = yaw-self.init_setvalue_yaw



        pt1=Point(self.car_x,self.car_y)
        pt2=Point(self.path[self.corner_num,0],self.path[self.corner_num,1])
        #need to change this spee!
        # v=8.0
        
        self.point2point(pt1,pt2,yaw)

        rospy.loginfo("destflag %f"%self.destflag)
        rospy.loginfo("startflag %f"%self.startflag)
        if self.destflag==True:
            self.corner_num=self.corner_num+1
            if self.corner_num==len(self.path):
                self.corner_num=0
        # print("corner_num")
        # print(corner_num)
        # motion,lineflag,lastlineflag = waypoint_follower(lineflag, lastlineflag, position,yaw)
        

            #decrease the speed here
        if self.nearest_reading <= self.nearestreading_stop:
            self.v = 0.0
        print("nearest_reading")
        print(self.nearest_reading)
        fid.write("%s\n" %self.nearest_reading)

        self.out_v=self.v
        self.out_w=self.w

    def callback2(self,msg):#velocity test
        linear=msg.twist.linear
        angular=msg.twist.angular

        self.car_ctrlSpeed=math.sqrt(linear.x**2+linear.y**2+linear.z**2)
        self.car_ctrlSteer=angular.z

    def callback3(self,msg):#scan and collision avoidance.
        ranges=msg.ranges
        self.stopflag=False
        self.readings=ranges
        for i in ranges[7:]:
            if i<5:
                self.stopflag=True

        # print(stopflag)
        self.nearest_reading = min(ranges)
        self.mytime = rospy.get_time()
        self.sick_readings.write("%2.2f, " %self.mytime)
        for value in ranges:
            self.sick_readings.write("%2.4f, " %value)

        self.sick_readings.write("\n")
        # print("nearest reading: %2.2f" %nearest_reading)
    def write_msg(self):
        self.results_file_handle.write("%2.6f %2.6f %2.6f %2.6f %2.6f %2.6f \n" %(rospy.get_time(), self.car_x, self.car_y, self.car_theta, self.car_ctrlSpeed, self.car_ctrlSteer))
        if self.readings:
            for value in self.readings:
                self.pedestrains_readings.write("%2.4f, " %value)
            self.pedestrains_readings.write("%2.6f %2.6f %2.6f %2.6f %2.6f %2.6f \n" %(rospy.get_time(), self.car_x, self.car_y, self.car_theta, self.car_ctrlSpeed, self.car_ctrlSteer))
        self.pedestrains_readings.write("\n")
def start():
    cardata=Car_data()
    pioneerdata=Pioneer_data()
    path_plan=path_planner()
    path_plan.set_vehicle_data(pioneerdata,cardata)
    motion = Twist()
    rospy.init_node("car_controller")
    # print("got")
    # x=input("xxxx")
    r=rospy.Rate(path_plan.freq)#60hz


    while not rospy.is_shutdown():
        # results_file_handle = open("Published_results.dat","a")
        rospy.loginfo("Time:%fIn the loop,my speed:%f,my angular velocity:%f"%(rospy.get_time(),path_plan.out_v,path_plan.out_w))
        motion.linear.x=path_plan.out_v
        motion.angular.z=path_plan.out_w
        print("motion")
        print(motion.linear.x)
        print("w")
        print(motion.angular.z)

        path_plan.write_msg()
        #for testing the speed
        # motion.linear.x=1
        # motion.angular.z=0.1
        path_plan.cmd.publish(motion)
        r.sleep()
        # rospy.Subscriber("/robot/velocity", TwistStamped, callback2)
    rospy.spin()

if __name__ == '__main__':
    start()#     listener()