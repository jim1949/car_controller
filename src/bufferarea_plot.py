#!/usr/bin/env python
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
from matplotlib import gridspec

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
    




bufferarea = np.loadtxt("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/bufferarea.dat")
#self.bufferarea.write("%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f \n "%(rospy.get_time(),estimation.pose_x,estimation.pose_y,estimation.v_x,motionstate.rec[0],motionstate.rec[1],motionstate.rec[2],motionstate.rec[3],self.car_x,self.car_y,self.car_ctrlSpeed))
sickreadings=np.loadtxt("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pod__sick_data.dat")
#1.get data from the file
# local=sickreadings[:,(1,180)]
world_x=sickreadings[:,range(0,180)]
world_y=sickreadings[:,range(180,360)]


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

a=bufferarea[:,15]
jerk=bufferarea[:,16]
#distance from true to observe
error=bufferarea[:,17]

k_pose_x=bufferarea[:,18]
k_pose_y=bufferarea[:,19]

error_k=[]
error_x=[]
error_y=[]
for i in range(0,len(k_pose_x)): 
    error_x.append(k_pose_x[i]-observe_pose_x[i])
    error_y.append(k_pose_y[i]-observe_pose_y[i])
    # print("error_x:%f,kalman_x:%f,obs_x:%f\n"%(error_x[i],kalman_x[i],observed_x[i]))
    # print("error_y:%f,kalman_y:%f,obs_y:%f\n"%(error_y[i],kalman_y[i],observed_y[i]))
for i in range(0,len(error_x)):
    error_k.append(np.sqrt(error_x[i]*error_x[i]+error_y[i]*error_y[i]))
# error=[math.sqrt(a*a+b*b) for a,b in zip(kalman_x,kalman_y)]
# figure 1
# print(error)
# compare their errors (kalman and particle)

# 1.filter error
plt.figure()
plt.plot(t,error_k,'r--',label="$kalman filter$",linewidth=2)
plt.plot(t,error,'b-.',label="$particle filter$",linewidth=2)
plt.xlabel("$Time (s)$")
plt.ylabel("$Error$ $distance (m)$")
plt.title("Kalman filter and Particle filter results")
plt.legend()
plt.show()


#2.filter estimation position.
plt.figure()
l1=plt.scatter(k_pose_x,k_pose_y, marker='x',color='r')
l2=plt.scatter(estimation_pose_x,estimation_pose_y,marker='x',color='b')
l3=plt.scatter(real_pose_x,real_pose_y,marker='.',color='y')
l4=plt.scatter(observe_pose_x,observe_pose_y,marker='.',color='c')
plt.xlabel("$x$ $axis$")
plt.ylabel("$y$ $axis$")
plt.title("Kalman filter and Particle filter results")
plt.legend((l1,l2,l3,l4),
           ('Kalman filter', 'Particle filter', 'real position','observe position'),
           scatterpoints=1,
           loc='upper right',
           ncol=3,
           fontsize=8)
plt.show()








speederror=estimation_v_x-car_ctrlSpeed

# car_rec_x=[-0.5,0.5]
# car_rec_y=[-0.5,0.5]
# p_rec_x=[-0.5,0.5]
# p_rec_y=[-0.5,0.5]

# print(world_x[19][20])


# print(car_x)
#2.plot 
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# fig=plt.figure()
# for i in range(0,len(t)):
#     for j in range(0,180):
#         ax.scatter(world_x[i,j],world_y[i,j],t[i])
# plt.show()


# plt.scatter(t,car_x,car_y,color='red',s=10,edgecolor='blue')
#plot location of the car_vehilce
# ax.scatter(car_x, car_y, t, c='r', marker='o')

#plot the 3D model
# fig = plt.figure()
# ax = fig.gca(projection='3d')
# ax.set_aspect("auto")
# ax.set_autoscale_on(True)

# ax=plot_3d(ax,car_x,car_y,t,car_rec_x,car_rec_y,'y','y')
# ax=plot_3d(ax,real_pose_x,real_pose_y,t,p_rec_x,p_rec_y,'red','b')
# plt.show()

# fig = plt.figure()
# ax1 = fig.gca(projection='3d')
# ax1.set_aspect("auto")
# ax1.set_autoscale_on(True)
# ax1=plot_3d(ax1,x_u,x_l,t,y_l,y_r,'red','red')
# # ax.scatter(estimation_pose_x,estimation_pose_y,t,c='b', marker='o')

# # ax1=plot_3d(estimation_pose_x,estimation_pose_y,t,p_rec_x,p_rec_y)

# plt.show()

# for i in range(0,180):
#     plt.scatter(world_x,world_y,s=1,c='blue',alpha=0.5)
# plt.show()

# #3.car position, speed, acceleration, jerk
# fig = plt.figure()

# ax0=fig.add_subplot(411)
# ax0.plot(t,car_y)
# ax0.set_xlim(0,50)
# ax0.set_ylabel("car position(m)")
# # ax0.title.set_text("comparison")

# ax1 = fig.add_subplot(412)
# ax1.plot(t,car_ctrlSpeed)
# ax1.set_xlim(0,50)
# ax1.set_ylabel("car speed(m/s)")

# ax2 = fig.add_subplot(413)
# ax2.plot(t,a)
# ax2.set_xlim(0,50)
# ax2.set_ylabel("car acceleration(m/s^2)")


# ax3 = fig.add_subplot(414)
# ax3.plot(t,jerk)
# ax3.set_xlim(0,50)
# ax3.set_ylabel("car jerk (m/s^3)")

# plt.plot()
# plt.show()
# fig = plt.gcf()

#4.error plot
fig1=plt.figure()
ax1=fig1.add_subplot(211)
ax1.plot(t,error)
ax1.set_xlim(0,50)
ax1.set_ylabel("estimation distance error")

ax2=fig1.add_subplot(212)
ax2.plot(t,speederror)
ax2.set_xlim(0,50)
ax2.set_ylabel("estimation speed error")
plt.plot()
plt.show()



# plotly_fig = tls.mpl_to_plotly( fig )
# plotly_fig['layout']['title'] = 'Comparison of v,a,jerk'
# plotly_fig['layout']['margin'].update({'t':40})

# plot_url = py.plot(plotly_fig, filename='mpl-simple-subplot')
# #3.plot for 3D






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