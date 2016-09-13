#!/usr/bin/env python
import numpy as np
from sklearn.cluster import MeanShift, estimate_bandwidth
from sklearn.datasets.samples_generator import make_blobs

###############################################################################

# Plot result
import matplotlib.pyplot as plt
from itertools import cycle




bufferarea = np.loadtxt("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/bufferarea.dat")
#self.bufferarea.write("%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f \n "%(rospy.get_time(),estimation.pose_x,estimation.pose_y,estimation.v_x,motionstate.rec[0],motionstate.rec[1],motionstate.rec[2],motionstate.rec[3],self.car_x,self.car_y,self.car_ctrlSpeed))
sickreadings=np.loadtxt("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pod__sick_data.dat")
#1.get data from the file
# local=sickreadings[:,(1,180)]
world_x=sickreadings[:,(0,179)]
world_y=sickreadings[:,(180,359)]


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


# Generate sample data
centers = [[1, 1], [-1, -1], [1, -1]]
X, _ = make_blobs(n_samples=10000, centers=centers, cluster_std=0.6)
X=[observe_pose_x,observe_pose_y]
###############################################################################
# Compute clustering with MeanShift

# The following bandwidth can be automatically detected using
bandwidth = estimate_bandwidth(X, quantile=0.2, n_samples=500)

ms = MeanShift(bandwidth=bandwidth, bin_seeding=True)
ms.fit(X)
labels = ms.labels_
cluster_centers = ms.cluster_centers_

labels_unique = np.unique(labels)
n_clusters_ = len(labels_unique)

print("number of estimated clusters : %d" % n_clusters_)

plt.figure(1)
plt.clf()

colors = cycle('bgrcmykbgrcmykbgrcmykbgrcmyk')
for k, col in zip(range(n_clusters_), colors):
    my_members = labels == k
    cluster_center = cluster_centers[k]
    plt.plot(X[my_members, 0], X[my_members, 1], col + '.')
    plt.plot(cluster_center[0], cluster_center[1], 'o', markerfacecolor=col,
             markeredgecolor='k', markersize=14)
plt.title('Estimated number of clusters: %d' % n_clusters_)
plt.show()