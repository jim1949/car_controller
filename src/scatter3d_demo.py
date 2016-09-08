# import numpy as np
# from mpl_toolkits.mplot3d import Axes3D
# import matplotlib.pyplot as plt


# # def randrange(n, vmin, vmax):
# #     return (vmax - vmin)*np.random.rand(n) + vmin

# # fig = plt.figure()
# # ax = fig.add_subplot(111, projection='3d')
# # n = 100
# # for c, m, zl, zh in [('r', 'o', -50, -25), ('b', '^', -30, -5)]:
# #     xs = randrange(n, 23, 32)
# #     ys = randrange(n, 0, 100)
# #     zs = randrange(n, zl, zh)
# #     ax.scatter(xs, ys, zs, c=c, marker=m)
# xs=np.array((range(1,100)))
# ys=np.array((range(1,100)))
# zs=np.array((range(1,100)))
# Axes3D.scatter(xs, ys, zs, zdir='z',  c='b')

# ax.set_xlabel('X Label')
# ax.set_ylabel('Y Label')
# ax.set_zlabel('Z Label')

# plt.show()

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt



fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

x =[1,2,3,4,5,6,7,8,9,10]
y =[5,6,2,3,13,4,1,2,4,8]
z =[2,3,3,3,5,7,9,11,9,10]



ax.scatter(x, y, z, c='r', marker='o')

ax.set_xlabel('X Label')
ax.set_ylabel('Y Label')
ax.set_zlabel('Z Label')

plt.show()
