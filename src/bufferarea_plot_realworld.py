#!/usr/bin/env python
#real world
import numpy as np
import matplotlib.pyplot as plt
import pylab as pl
import random
import time
import math

from mpl_toolkits.mplot3d import Axes3D,art3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib import cm
from itertools import product, combinations

def plot_3d(ax,x,y,z,rec_x,rec_y,color1,color2):

    recx=[]
    recy=[]

    for i in range(0,len(x)):
        recx.append(rec_x+x[i]*np.ones(2))
        recy.append(rec_y+y[i]*np.ones(2))

    for t in range(0,len(z)):
        m=z[t]
        btm=np.array([[recx[t][0],recy[t][0],m],
                    [recx[t][0],recy[t][1],m],
                    [recx[t][1],recy[t][1],m],
                    [recx[t][1],recy[t][0],m]])
        # print(btm)
        side = art3d.Poly3DCollection([btm])
        side.set_color(color1)
        side.set_facecolor(color2)
        # side.set_alpha(0.5)
        ax.add_collection3d(side)
    ax.set_xlim3d(-5,5)
    ax.set_ylim3d(-15,2)
    ax.set_zlim3d(0,z[-1])
    ax.set_xlabel("x axis")
    ax.set_ylabel("y axis")
    ax.set_zlabel("time") 
    return ax  
    


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

bufferarea = np.loadtxt("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/bufferarea.dat")
#self.bufferarea.write("%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f \n "%(rospy.get_time(),estimation.pose_x,estimation.pose_y,estimation.v_x,motionstate.rec[0],motionstate.rec[1],motionstate.rec[2],motionstate.rec[3],self.car_x,self.car_y,self.car_ctrlSpeed))

#1.get data from the file
t=bufferarea[:,0]
t=t-bufferarea[0][0]
estimation_pose_x=bufferarea[:,1]
estimation_pose_y=bufferarea[:,2]
estimation_v_x=bufferarea[:,3]
x_u=bufferarea[:,4]
x_l=bufferarea[:,5]
y_l=bufferarea[:,6]
y_r=bufferarea[:,7]

car_x=bufferarea[:,8]
car_y=bufferarea[:,9]
car_ctrlSpeed=bufferarea[:,10]
observe_pose_x=bufferarea[:,11]
observe_pose_y=bufferarea[:,12]

real_pose_x=bufferarea[:,13]
real_pose_y=bufferarea[:,14]
speederror=estimation_v_x-car_ctrlSpeed

car_rec_x=[-0.25,0.25]
car_rec_y=[-0.25,0.25]
p_rec_x=[-0.25,0.25]
p_rec_y=[-0.25,0.25]

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_aspect("auto")
ax.set_autoscale_on(True)
ax=plot_3d(ax,car_x,car_y,t,car_rec_x,car_rec_y,'y','y')
# print(car_x)
#2.plot 



# plt.scatter(t,car_x,car_y,color='red',s=10,edgecolor='blue')
#plot location of the car_vehilce
# ax.scatter(car_x, car_y, t, c='r', marker='o')

ax=plot_3d(ax,estimation_pose_x,estimation_pose_y,t,p_rec_x,p_rec_y,'red','b')



# ax.scatter(estimation_pose_x,estimation_pose_y,t,c='b', marker='o')

# ax1=plot_3d(estimation_pose_x,estimation_pose_y,t,p_rec_x,p_rec_y)

plt.show()

#3.plot for 3D




# ax.set_xlabel('X Label')
# ax.set_ylabel('Y Label')
# ax.set_zlabel('Time')

# plt.xlim([-3,3])

# plt.show()





# print(car_x)
# print(car_y)
# car_speed=cardata[:,4]
# car_speed_average=sum(car_speed)/len(car_speed)
# # plt.scatter(car_y,car_speed,color='blue',s=8,edgecolor='none')
# plt.plot(car_y,car_speed)


# plt.plot(car_y,car_speed_average*np.ones(len(car_speed)))
# plt.xlim([0.0,-10.0])
# plt.show()
# plt.plot(t,car_speed)
# plt.show()