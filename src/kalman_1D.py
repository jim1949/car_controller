# Kalman filter example demo in Python

# A Python implementation of the example given in pages 11-15 of "An
# Introduction to the Kalman Filter" by Greg Welch and Gary Bishop,
# University of North Carolina at Chapel Hill, Department of Computer
# Science, TR 95-041,
# http://www.cs.unc.edu/~welch/kalman/kalmanIntro.html

# by Andrew D. Straw

import numpy as np
import matplotlib.pyplot as plt

pose = np.loadtxt("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/people_position.dat")

# N = 20
# true_x = np.linspace(0.0, 10.0, N)
# true_y = true_x**3
# print(pose)
# observed_x = true_x + 0.05*np.random.random(N)*true_x
# observed_y = true_y + 0.05*np.random.random(N)*true_y

true_x=[]
true_y=[]
observed_x=[]
observed_y=[]

for i in range(len(pose)):
    # print(i)
    true_x.append(pose[i-1][2])
    true_y.append(pose[i-1][3]-5)
    observed_x.append(pose[i-1][0])
    observed_y.append(pose[i-1][1])

plt.rcParams['figure.figsize'] = (10, 8)

# intial parameters
n_iter = len(true_x)
sz = (n_iter,) # size of array
x = np.array(true_x) # truth value (typo in example at top of p. 13 calls this z)
z = observed_x # observations (normal about x, sigma=0.1)

Q = 1e-5 # process variance

# allocate space for arrays
xhat=np.zeros(sz)      # a posteri estimate of x
P=np.zeros(sz)         # a posteri error estimate
xhatminus=np.zeros(sz) # a priori estimate of x
Pminus=np.zeros(sz)    # a priori error estimate
K=np.zeros(sz)         # gain or blending factor

R = 0.1**2 # estimate of measurement variance, change to see effect

# intial guesses
xhat[0] = 0.0
P[0] = 1.0

for k in range(1,n_iter):
    # time update
    xhatminus[k] = xhat[k-1]
    Pminus[k] = P[k-1]+Q

    # measurement update
    K[k] = Pminus[k]/( Pminus[k]+R )
    xhat[k] = xhatminus[k]+K[k]*(z[k]-xhatminus[k])
    P[k] = (1-K[k])*Pminus[k]

plt.figure()
plt.plot(z,'k+',label='noisy measurements')
plt.plot(xhat,'b-',label='a posteri estimate')
plt.plot(x,color='g',label='truth value')
plt.legend()
plt.title('Estimate vs. iteration step', fontweight='bold')
plt.xlabel('Iteration')
plt.ylabel('x axis')

plt.figure()
valid_iter = range(1,n_iter) # Pminus not valid at step 0
plt.plot(valid_iter,Pminus[valid_iter],label='a priori error estimate')
plt.title('Estimated $\it{\mathbf{a \ priori}}$ error vs. iteration step', fontweight='bold')
plt.xlabel('Iteration')
plt.ylabel('x axis')
plt.setp(plt.gca(),'ylim',[0,.01])
plt.show()