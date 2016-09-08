from mpl_toolkits.mplot3d import Axes3D, art3d
import matplotlib.pyplot as plt
import numpy as np
from itertools import product, combinations
from numpy import sin, cos


fig = plt.figure()
ax = fig.gca(projection='3d')
ax.set_aspect("auto")
ax.set_autoscale_on(True)


r = [-10, 10]
for s, e in combinations(np.array(list(product(r,r,r))), 2):
    if np.sum(np.abs(s-e)) == r[1]-r[0]:
        ax.plot3D(*zip(s,e), color="b")


btm = np.array([[-2, -2, -2],
                [-2, 2, -2],
                [ 2, 2, -2],
                [2, -2,-2]])
top = np.array([[-2, -2, 2],
                [-2, 2, 2],
                [ 2, 2, 2],
                [2, -2,2]])
# theta = np.radians(45) 
# rot_mx = np.array([[cos(theta), sin(theta), 0],
#                     [-sin(theta), cos(theta), 0],
#                     [          0,          0, 1]])
x=[1,2,3]
y=[0,0,0]
z=[1,2,6]
btm=np.array([x,y,z])
# btm = np.dot(btm, rot_mx)
side1 = art3d.Poly3DCollection([btm],alpha=0.5, linewidth=1)

side1.set_color('r')
alpha = 0.5
side1.set_facecolor((1, 0, 0, alpha))

ax.add_collection3d(side1)

# top = np.dot(top, rot_mx)
side2 = art3d.Poly3DCollection([top])
side2.set_color('g')
side2.set_facecolor((0, 1, 0, alpha))
ax.add_collection3d(side2)


plt.show()