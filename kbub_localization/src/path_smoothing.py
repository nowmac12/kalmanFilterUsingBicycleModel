#!/usr/bin/env python3
from math import *
import os,sys
import matplotlib.pyplot as plt
import rospy


rospy.init_node('path_smoother',anonymous=True)

def plotter(cx,cy,title):
    plt.figure(title)
    plt.plot(cx,cy,'.b')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.axis("equal")
    plt.grid(True)

# ------------------------------------------------
# smooth coordinates
#

def smooth(path, weight_data = 0.01, weight_smooth = 0.27, tolerance = 0.00001):

    # Make a deep copy of path into newpath
    newpath = [[0 for col in range(len(path[0]))] for row in range(len(path))]
    for i in range(len(path)):
        for j in range(len(path[0])):
            newpath[i][j] = path[i][j]

    #### ENTER CODE BELOW THIS LINE ###
    change = 1
    while change > tolerance:
        change = 0
        for i in range(1,len(path)-1):
            for j in range(len(path[0])):
                ori = newpath[i][j]
                newpath[i][j] = newpath[i][j] + weight_data*(path[i][j]-newpath[i][j])
                newpath[i][j] = newpath[i][j] + weight_smooth*(newpath[i+1][j]+newpath[i-1][j]-2*newpath[i][j])
                change += abs(ori - newpath[i][j])
    
    return newpath # Leave this line for the grader!

# feel free to leave this and the following lines if you want to print.
path = '/home/'+os.getlogin()+'/catkin_ws/src/txt_saver/map/' 
file_list = os.listdir(path) 


#------------------PREPROCESSING-------------------#
old_cx, old_cy, new_cx, new_cy = [], [], [], []
rospy.logwarn("Waiting for PATH SMOOTHING....DO NOT EXIT PROGRAM")
txts = [file for file in file_list if file.endswith(".txt")]
for txt in txts:
    cx =[]
    pw = open('/home/'+os.getlogin()+'/catkin_ws/src/txt_saver/map/' + txt, 'r')
    while True:
        line = pw.readline()
        if not line:
            break
        if line.find('\n'):
            line = line[:line.find('\n')]
            temp = list(map(float,line.split(',')))
        cx.append(temp)
        old_cx.append(temp[0])
        old_cy.append(temp[1])
    pw.close()
    newpath = smooth(cx)
    rospy.loginfo("Current text file is {}".format(txt))
    
    with open('/home/'+os.getlogin()+'/catkin_ws/src/txt_saver/map/' +txt, 'w') as fw:
        for coord_x,coord_y in newpath:
            # print(coord_x,coord_y)
            fw.write(str(coord_x))
            fw.write(',')
            fw.write(str(coord_y))
            fw.write('\n')
            new_cx.append(coord_x)
            new_cy.append(coord_y)
rospy.loginfo("DONE!")
rospy.loginfo("----PLOTTING GRAPH----")

plotter(old_cx,old_cy,'before')
plotter(new_cx,new_cy,'after')

plt.figure('compare')
plt.plot(old_cx,old_cy,'.b',label='Original')
plt.plot(new_cx,new_cy,'.r',label='Path Smoothing')
plt.xlabel('x')
plt.ylabel('y')
plt.axis("equal")
plt.grid(True)

plt.show()
