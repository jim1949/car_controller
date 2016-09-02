import numpy as np
import matplotlib.pyplot as plt

#self.results_file_handle.write("%2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f %2.4f\n" %(rospy.get_time(), self.car_x, self.car_y, self.car_theta, self.car_ctrlSpeed, self.car_ctrlSteer, peoplestates.world_nearest_position_x,peoplestates.world_nearest_position_y))

pose_file = np.loadtxt("/Users/jj/car_controller_ws/src/car_controller/src/data/test2/pioneer_simulation_cardata.dat")
pose=array(pose_file)
speed=pose[:,1]
distance=pose[:,4]
plt.plot(distance,speed,'go')
plt.show
