import numpy as np
import matplotlib.pyplot as plt
# Step 1: Sensor Noise
Quad_GPS_X = np.loadtxt('../config/log/Graph1.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]
Quad_IMU_AX = np.loadtxt('../config/log/Graph2.txt',delimiter=',',dtype='Float64',skiprows=1)[:,1]
GPS_X_std = np.std(Quad_GPS_X)
print('GPS X Std: {}'.format(GPS_X_std))
IMU_AX_std = np.std(Quad_IMU_AX)
print('IMU X Std: {}'.format(IMU_AX_std))
