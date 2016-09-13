import numpy as np
import matplotlib.pyplot as plt
import math
# from sklearn.metrics import mean_squared_error

def kalman_xy(x, P, measurement, R,
              motion = np.matrix('0. 0. 0. 0.').T,
              Q = np.matrix(np.eye(4))):
    """
    Parameters:    
    x: initial state 4-tuple of location and velocity: (x0, x1, x0_dot, x1_dot)
    P: initial uncertainty convariance matrix
    measurement: observed position
    R: measurement noise 
    motion: external motion added to state vector x
    Q: motion noise (same shape as P)
    """
    return kalman(x, P, measurement, R, motion, Q,
                  F = np.matrix('''
                      1. 0. 1. 0.;
                      0. 1. 0. 1.;
                      0. 0. 1. 0.;
                      0. 0. 0. 1.
                      '''),
                  H = np.matrix('''
                      1. 0. 0. 0.;
                      0. 1. 0. 0.'''))

def kalman(x, P, measurement, R, motion, Q, F, H):
    '''
    Parameters:
    x: initial state
    P: initial uncertainty convariance matrix
    measurement: observed position (same shape as H*x)
    R: measurement noise (same shape as H)
    motion: external motion added to state vector x
    Q: motion noise (same shape as P)
    F: next state function: x_prime = F*x
    H: measurement function: position = H*x

    Return: the updated and predicted new values for (x, P)

    See also http://en.wikipedia.org/wiki/Kalman_filter

    This version of kalman can be applied to many different situations by
    appropriately defining F and H 
    '''
    # UPDATE x, P based on measurement m    
    # distance between measured and current position-belief
    y = np.matrix(measurement).T - H * x
    S = H * P * H.T + R  # residual convariance
    K = P * H.T * S.I    # Kalman gain
    x = x + K*y
    I = np.matrix(np.eye(F.shape[0])) # identity matrix
    P = (I - K*H)*P

    # PREDICT x, P based on motion
    x = F*x + motion
    P = F*P*F.T + Q

    return x, P

def demo_kalman_xy(observed_x,observed_y,x,P,result):


    
    # N = 20
    # true_x = np.linspace(0.0, 10.0, N)
    # true_y = true_x**3
    # print(pose)
    # observed_x = true_x + 0.05*np.random.random(N)*true_x
    # observed_y = true_y + 0.05*np.random.random(N)*true_y

    # true_x=[]
    # true_y=[]
    # observed_x=[]
    # observed_y=[]

    # for i in range(len(pose)):
    #     # print(i)
    #     true_x.append(pose[i-1][2])
    #     true_y.append(pose[i-1][3]-5)
    #     observed_x.append(pose[i-1][0])
    #     observed_y.append(pose[i-1][1])


    # true_x=pose[:,13]
    # true_y=pose[:,14]
    # observed_x=pose[:,11]
    # observed_y=pose[:,12]

    
    R = 0.01**2

    meas=(observed_x, observed_y)
    x, P = kalman_xy(x, P, meas, R)
    # print(P)
    # print("\n")
    result.append((x[:2]).tolist())
    # print(*result)

    # kalman_x.append(kalmanx)
    # kalman_y.append(kalmany)

    
    kalmanx,kalmany=zip(*result)
    # print(x[0])
    print(max(kalmany[-1]))
    # print(x[1])
    print(max(kalmany[-1]))

    plt.scatter(kalmanx[-1],kalmany[-1],s=1,c='red',alpha=0.5)
    plt.scatter(observed_x,observed_y,s=1,c='blue',alpha=0.5)
    return x,P,result


   

pose = np.loadtxt("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/bufferarea.dat")
true_x=pose[:,13]
true_y=pose[:,14]
observed_x=pose[:,11]
observed_y=pose[:,12]
x = np.matrix('0. 0. 0. 0.').T 
P = np.matrix(np.eye(4))*10000 # initial uncertainty

result = []
plt.figure()
for i in range(0,len(observed_x)):

    x,P,result=demo_kalman_xy(observed_x[i],observed_y[i],x,P,result)
plt.xlim(-8,8)
plt.ylim(-10,10)
plt.show()
kalman_x, kalman_y = zip(*result)
kalman_x=np.array(kalman_x)
kalman_y=np.array(kalman_y)






error=[]
error_x=[]
error_y=[]
for i in range(0,len(kalman_x)): 
    error_x.append(kalman_x[i]-observed_x[i])
    error_y.append(kalman_y[i]-observed_y[i])
    # print("error_x:%f,kalman_x:%f,obs_x:%f\n"%(error_x[i],kalman_x[i],observed_x[i]))
    # print("error_y:%f,kalman_y:%f,obs_y:%f\n"%(error_y[i],kalman_y[i],observed_y[i]))
for i in range(0,len(error_x)):
    error.append(np.sqrt(error_x[i]*error_x[i]+error_y[i]*error_y[i]))
# error=[math.sqrt(a*a+b*b) for a,b in zip(kalman_x,kalman_y)]
# figure 1
# print(error)



plt.figure()
plt.plot(range(0,len(error)),error,'r-',label="$kalman error$",linewidth=2)
plt.xlabel("$Time$")
plt.ylabel("$Error $")
plt.title("Kalman filter Error results")
plt.legend()
plt.show()
# print(len(pose))

plt.figure()
plt.plot(kalman_x, kalman_y,"b--",label="$estimate positions$",linewidth=2)
plt.plot(observed_x, observed_y,'r-.',label="$observation positions$", linewidth=2)
plt.plot(true_x,true_y,'g--',label="$true positions$",linewidth=1)

# plt.plot(kalman_x, kalman_y,"bo")

# print(kalman_x)
# print(kalman_y)

plt.xlabel("$x axis$")
plt.ylabel("$y axis$")
plt.title("Kalman filter results")
plt.legend()
# plt.plot(kalman_x, kalman_y, 'g-')
plt.show()