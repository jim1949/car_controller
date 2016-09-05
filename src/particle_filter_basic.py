#based on Sampling Importance Resampling (SIR)
# in case something may come inside of the map, still need to maintain lots of particles inside of the map.

#always in the area.
import numpy as np
import matplotlib.pyplot as plt
import pylab as pl
import random
import time
import math

# N = 20
# true_x = np.linspace(0.0, 10.0, N)
# true_y = true_x**3
# print(pose)
# observed_x = true_x + 0.05*np.random.random(N)*true_x
# observed_y = true_y + 0.05*np.random.random(N)*true_y
# pl.ion()
global ranges_num
#need change sometimes
ranges_num=37
class measurement():
    def __init__(self):
        self.pose_xy = np.loadtxt("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/people_position.dat")
        #every two rows is position x row, and position y row
        self.pose_x=[]
        self.pose_y=[]

        self.pose_x_mean=0.0
        self.pose_y_mean=0.0

        self.pose_x_last_mean=0.0
        self.pose_x_last_mean=0.0

        if len(self.pose_xy)%2==1:
            print("the data is odd!!!!")
        for i in range(0,int(len(self.pose_xy)/2)):
            self.pose_x.append(self.pose_xy[2*i])
            self.pose_y.append(self.pose_xy[2*i+1])
        #exact position: pose_x[i],pose_y[i]

        self.init()
        
    def init(self):
        self.pose_measure_buff_x=[]
        self.pose_measure_buff_y=[]
        self.point_no=0


    def generate(self,t,detectarea):
        self.init()
        i=t
        #t is 迭代次数
        sum_x=0.0
        sum_y=0.0
        for j in range(0,ranges_num):
            if self.pose_x[i][j]!=float('inf') and self.pose_y[i][j]!=float('inf'):
                self.pose_measure_buff_x.append(self.pose_x[i][j])
                self.pose_measure_buff_y.append(self.pose_y[i][j])
                sum_x=sum_x+self.pose_x[i][j]
                sum_y=sum_y+self.pose_y[i][j]
                self.point_no=self.point_no+1

        if self.point_no!=0:
            self.pose_x_mean=sum_x/self.point_no
            self.pose_y_mean=sum_y/self.point_no
        else:
            self.pose_x_mean=1e7
            self.pose_y_mean=1e7
        plt.figure(2)
        plt.scatter(self.pose_measure_buff_x,self.pose_measure_buff_y,color='blue',s=5,edgecolor='none')
        plt.scatter(self.pose_x_mean,self.pose_y_mean,color='blue',s=8,edgecolor='none')
        plt.xlabel("xaxis")
        plt.ylabel("yaxis")
        plt.xlim([detectarea.rec[0],detectarea.rec[1]])
        plt.ylim([detectarea.rec[2],detectarea.rec[3]])
        plt.title("mean point and measurement points")
        # plt.show()
    



        #output:point_no, current point positions 
    def update(self):
        self.pose_x_last_mean=self.pose_x_mean
        self.pose_y_last_mean=self.pose_y_mean


class initial_data():
    def __init__(self,detectarea):  
        self.pose = np.loadtxt("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/people_position_simulation.dat") 
        
        self.true_x=[]
        self.true_y=[]
        self.observed_x=[]
        self.observed_y=[]
        #from t=0 to t=len(true_x)
        self.estimate_x=[]
        #from t=1 to t=len(true_x)+1
        self.estimate_y=[]

        self.possibility=[]
        self.t=0

        self.v_x=0

        self.particles_number=100

#1.initial part, feature extraction.

        for i in range(len(self.pose)):
            # print(i)
            self.true_x.append(self.pose[i-1][2])
            self.true_y.append(self.pose[i-1][3]-5)
            # self.true_x.append(self.pose_xy[i/2][])
            self.observed_x.append(self.pose[i-1][0])
            self.observed_y.append(self.pose[i-1][1])

        plt.scatter(self.true_x,self.true_y,color='red',s=5,edgecolor='none')
        plt.scatter(self.observed_x,self.observed_y,color='blue',s=5,edgecolor='none')
        plt.title("observed position vs true position")
        plt.xlabel("xaxis")
        plt.ylabel("yaxis")
        plt.xlim([detectarea.rec[0],detectarea.rec[1]])
        plt.ylim([detectarea.rec[2],detectarea.rec[3]])
        # plt.show()

class particle:
    def __init__(self,svs,x_dsampleSize):
        #particle states here.
        self.svs=svs
        self.x_dsampleSize=x_dsampleSize
    def likelihood(self):
        pass
    def distance_sq(self,x1,y1,x2,y2):
        distancesq=((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
        return distancesq

class detect_area:
    def __init__(self,x_t,x_d,y_l,y_r):
        self.rec=[x_t,x_d,y_l,y_r]

# class Likelihood:
#     def __init__(self):
# class Likelihood():
#     def __init__(self,model):
#         self.model=model
#     def generate(self,)
class Estimation():
    def __init__(self):
        self.pose_x=0.0
        self.pose_y=0.0
        self.pose_x_posterori=0.0
        self.pose_y_posterori=0.0
        self.v_x=0.0
        self.v_y=0.0
        self.estimate_x=0.0
        self.estimate_y=0.0

    def estimate_velocity(self,measure,dt,t):
        if t==0:
            self.v_x=0.0
            self.v_y=0.0
        else:
            self.v_x=(measure.pose_x_mean-measure.pose_x_last_mean)/dt
            self.v_y=(measure.pose_y_mean-measure.pose_y_last_mean)/dt



class Weight():
    def __init__(self,Particle_info):
        self.update(Particle_info)
    def update(self,Particle_info):
        self.gauss=np.zeros(Particle_info.x_dsampleSize)
        self.particle_gauss_no=np.zeros(Particle_info.x_dsampleSize)
        self.sum_gauss=0.0
        self.i=0
    def gauss_possiblity(self,Particle_info,measure):
        sigma=0.15#need to adjust this parameter
        self.update(Particle_info)
        for i in range(0,Particle_info.x_dsampleSize):
            distancesq=Particle_info.distance_sq(Particle_info.svs[i][0],Particle_info.svs[i][1],measure.pose_x_mean,measure.pose_y_mean)

            self.gauss[i]=np.exp(-0.5*distancesq/(sigma*sigma)) / (np.sqrt(2*np.pi)*sigma)

        self.sum_gauss=sum(self.gauss)
        print("sum_gauss%f:"%self.sum_gauss)

        # # print("i:%d"%self.i)

        print(self.gauss)
        return self.gauss

    def normalization(self,x_dsampleSize):

        for i in range(0,x_dsampleSize):

            self.gauss[i]=self.gauss[i]/self.sum_gauss
            self.particle_gauss_no[i]=int(x_dsampleSize*self.gauss[i])
        left_i=int(x_dsampleSize-sum(self.particle_gauss_no))
        # print(left_i)
        #usually here left_i>0,so we need to make sure it can be 0.
        #Adding some random particles to maintain the size as the same.
        left_array=[]
        #(1)store index of deleted particles in left_array.empty points...
        for i in range(0,x_dsampleSize):
            
            if self.particle_gauss_no[i]==0:
                left_array.append(i)
        # print("len:%d"%len(left_array))

        add_array=[]

        for i in range(0,left_i):
        #(2)random select the index of left_array, and save them ...random choose.
            # print("i:%d"%i)
            # print(len(left_array))
            
            x=random.randint(0,len(left_array)-1)
            add_array.append(left_array[x])
            del left_array[x]
        for i in add_array:
            self.particle_gauss_no[i]+=1

        # print("after normalization:")
        # print(sum(self.particle_gauss_no))

def resampling_gauss(particle,weight,x_dsampleSize,v_x,v_y,dt):

    #resampling process 1: at local area
    new_particle=[]
    for i in range(0,x_dsampleSize):
        if weight.particle_gauss_no[i]!=0:
            rec=[particle.svs[i][0]-0.3,particle.svs[i][0]+0.3,particle.svs[i][1]-0.3,particle.svs[i][1]+0.3]
            new_particle.extend(initStateVectors(rec,weight.particle_gauss_no[i]))
    particle.svs=new_particle

    # print(particle.svs)
    #resampling process 2: motion model
    new_particle=[]
    for i in range(0,x_dsampleSize):
        ##need to be careful here there is no dt.
        # print("i%d"%i)
        x1=particle.svs[i][0]+(v_x+random.uniform(0,0.3))*dt
        x2=particle.svs[i][0]-(v_x+random.uniform(0,0.3))*dt
        y1=particle.svs[i][1]+(v_x+random.uniform(0,0.3))*dt
        y2=particle.svs[i][1]-(v_x+random.uniform(0,0.3))*dt
        # print("x1:%fx2:%f,y1:%f,y2:%f"%(x1,x2,y1,y2))
        rec=[particle.svs[i][0]+(v_x+random.uniform(0,0.3))*dt,particle.svs[i][0]+(v_x-random.uniform(0,0.3))*dt,particle.svs[i][1]+(v_y+random.uniform(0,0.3))*dt,particle.svs[i][1]-(v_y+random.uniform(0,0.3))*dt]
        new_particle.extend(initStateVectors(rec,weight.particle_gauss_no[i]))
    particle.svs=new_particle

    #output:updated particle states



#initial particle position area
def initStateVectors(rec,sampleSize):
    sampleSize=int(sampleSize)
    xs = [random.uniform(rec[0],rec[1]) for i in range(sampleSize)]
    ys = [random.uniform(rec[2],rec[3]) for i in range(sampleSize)]
    vxs = [random.uniform(0,1) for i in range(sampleSize)]
    vys = [random.uniform(0,1) for i in range(sampleSize)]

    return([list(s) for s in zip(xs,ys,vxs,vys)])

#2.particle positioning.searching step./+tracking new particles, then we need to resample it!
# place dogs....
def start():
    #1).initialization
    detectarea=detect_area(-10.0,10.0,-.3,-7.0)
    initialdata=initial_data(detectarea)
    measure=measurement()
    t=0
    dt=1
    #in world frame. detect area

    
    #particle filter initialization

    
    #particle states
    x_dsampleSize=100
    Particle_info=particle(initStateVectors(detectarea.rec,x_dsampleSize),x_dsampleSize)


    plt.scatter(np.array(Particle_info.svs)[:,0],np.array(Particle_info.svs)[:,1],color='black',s=5,edgecolor='none')
    plt.xlabel('x axis')
    plt.ylabel('y axis')
    plt.xlim([detectarea.rec[0],detectarea.rec[1]])
    plt.ylim([detectarea.rec[2],detectarea.rec[3]])
    plt.title("particle initialization, random particles")
    # plt.show()
    # for t in range(0,len(initialdata.true_x)):
    RMES=[]
    for t in range(0,len(measure.pose_x)):

        measure.generate(t,detectarea)

    #2).calculate the weights by using gauss 
        #input:
        # particles positions,(np.array(particle.svs)[:,0],np.array(particle.svs)[:,1]) 
        # measurement positions,()
        #output:guass possibility for each of them. next time how many paricles will evolve from the original one.

        
        weight=Weight(Particle_info)
        weight.gauss_possiblity(Particle_info,measure)

        #Normalization
        weight.normalization( x_dsampleSize)

    #3).prior estimation (current estimation)
        estimation=Estimation()
        for i in range(0,x_dsampleSize):
            estimation.pose_x=Particle_info.svs[i][0]*weight.gauss[i]+estimation.pose_x
            estimation.pose_y=Particle_info.svs[i][1]*weight.gauss[i]+estimation.pose_y
        # estimation.v_x=estimation.pose_x-
        # estimation.v_y


        plt.scatter(estimation.pose_x,estimation.pose_y,color='black',s=8,edgecolor='none')
        plt.scatter(measure.pose_x_mean,measure.pose_y_mean,color='blue',s=8,edgecolor='none')
        plt.xlabel('x axis')
        plt.ylabel('y axis')       
        plt.xlim([detectarea.rec[0],detectarea.rec[1]])
        plt.ylim([detectarea.rec[2],detectarea.rec[3]])
        plt.title("prior estimation vs measure mean")
        # plt.show()

        RMES.append(math.sqrt((estimation.pose_x-measure.pose_x_mean)**2+(estimation.pose_y-measure.pose_y_mean)**2))


    #4).Resampling(twice along with the motion)
        estimation.estimate_velocity(measure,dt,t)


        resampling_gauss(Particle_info,weight,x_dsampleSize,estimation.v_x,estimation.v_y,dt)

        plt.scatter(np.array(Particle_info.svs)[:,0],np.array(Particle_info.svs)[:,1],color='black',s=5,edgecolor='none')
        plt.xlabel('x axis')
        plt.ylabel('y axis')
        plt.xlim([detectarea.rec[0],detectarea.rec[1]])
        plt.ylim([detectarea.rec[2],detectarea.rec[3]])
        plt.title("particle resampling, random particles")
        # plt.show()

        measure.update()

    #5) posterior estimation
        # estimation.pose_x_posterori=sum(self.Particle_info.svs[])
        estimation=Estimation()
        for i in range(0,x_dsampleSize):
            estimation.pose_x=Particle_info.svs[i][0]+estimation.pose_x
            estimation.pose_y=Particle_info.svs[i][1]+estimation.pose_y
        estimation.pose_x=estimation.pose_x/x_dsampleSize
        estimation.pose_y=estimation.pose_y/x_dsampleSize
    plt.figure(4)
    plt.plot(RMES)
    plt.show()

    #calculate the possibility for every particle.



    
    # plt.close("all")


    # plt.close()

if __name__ == '__main__':
    start()#     listener()
