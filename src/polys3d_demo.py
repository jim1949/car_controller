# from mpl_toolkits.mplot3d import Axes3D
# from matplotlib.collections import PolyCollection
# from matplotlib.colors import colorConverter
# import matplotlib.pyplot as plt
# import numpy as np


# fig = plt.figure()
# ax = fig.gca(projection='3d')


# def cc(arg):
#     return colorConverter.to_rgba(arg, alpha=0.6)

# xs = np.arange(0, 10, 0.4)
# verts = []
# zs = [0.0, 1.0, 2.0, 3.0]
# for z in zs:
#     ys = np.random.rand(len(xs))
#     ys[0], ys[-1] = 0, 0
#     verts.append(list(zip(xs, ys)))
# print(verts)
# poly = PolyCollection(verts, facecolors=[cc('r'), cc('g'), cc('b'),
#                                          cc('y')])
# poly = PolyCollection(verts, facecolors=cc('r'))
# poly.set_alpha(0.7)
# ax.add_collection3d(poly, zs=zs, zdir='y')

# ax.set_xlabel('X')
# ax.set_xlim3d(0, 10)
# ax.set_ylabel('Y')
# ax.set_ylim3d(-1, 4)
# ax.set_zlabel('Z')
# ax.set_zlim3d(0, 1)

# plt.show()
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

fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_aspect("auto")
ax.set_autoscale_on(True)
x=[0,1,2,3,4,5]
y=[0,1,2,3,6,7]
z=[0,1,2,3,4,5]

rec_x=[-1,1]
rec_y=[-1,1]
recx=[]
recy=[]

# r = [-10, 10]
# for s, e in combinations(np.array(list(product(r,r,r))), 2):
#     if np.sum(np.abs(s-e)) == r[1]-r[0]:
#         ax.plot3D(*zip(s,e), color="b")
for i in range(0,len(x)):
    recx.append(rec_x+x[i]*np.ones(2))
    recy.append(rec_y+y[i]*np.ones(2))
print(recx)
print(recy)
print(len(z))
print(z[0])
for t in range(0,len(z)):
    m=z[t]
    btm=np.array([[recx[t][0],recy[t][0],m],
                [recx[t][0],recy[t][1],m],
                [recx[t][1],recy[t][1],m],
                [recx[t][1],recy[t][0],m]])
    # btm = np.array([[recx[t][0], recy[t][0], -2],
    #             [recx[t][0], 2, -2],
    #             [ recx[t][1], 2, -2],
    #             [recx[t][1], -2,-2]])
    print(btm)
    side = art3d.Poly3DCollection([btm])
    side.set_color('r')
    side.set_facecolor('b')
    side.set_alpha(0.5)
    ax.add_collection3d(side)
ax.set_xlim3d(-5,5)
ax.set_ylim3d(-10,10)
ax.set_zlim3d(0,z[-1])
   
plt.show()


