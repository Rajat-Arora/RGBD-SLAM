# !/usr/bin/env python3 
#  coding=utf-8 

import  numpy as np
import  matplotlib.pyplot as plt
import  mpl_toolkits.mplot3d
from mpl_toolkits.mplot3d import Axes3D 

f  = open( "rgbd_dataset_freiburg1_room/groundtruth.txt" )
x  =  []
y  =  []
z  =  []
for  line  in  f:
    if  line[0] ==  ' # ' :
        continue 
    data  =  line.split()
    x.append(float(data[1]))
    y.append(float(data[2]))
    z.append(float(data[3]))

ax = plt.figure()
ax  = plt.subplot(111, projection='3d')
ax.plot(x,y,z)
plt.show()
