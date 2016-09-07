#!/usr/bin/env python
import numpy as np
import matplotlib.pyplot as plt
import pylab as pl
import random
import time
import math

cardata = np.loadtxt("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pioneer_simulation_cardata.dat")
t=cardata[:,0]
t=t-cardata[0][0]
car_x=cardata[:,1]
car_y=cardata[:,2]
print(car_x)
print(car_y)
car_speed=cardata[:,4]
car_speed_average=sum(car_speed)/len(car_speed)
# plt.scatter(car_y,car_speed,color='blue',s=8,edgecolor='none')
plt.plot(car_y,car_speed)


plt.plot(car_y,car_speed_average*np.ones(len(car_speed)))
plt.show()
plt.plot(t,car_speed)
plt.show()