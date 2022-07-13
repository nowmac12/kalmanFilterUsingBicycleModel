#!/usr/bin/env python

import math
import os, sys
import matplotlib.pyplot as plt
import numpy as np 

first = 1
def main(): 
    global first
    old_x, old_y,new_xs, new_ys = [], [],[],[]
    user_name = os.getlogin()
    file_path = "/home/"+user_name+"/catkin_ws/src/txt_saver/map/"
    file_lists = os.listdir(file_path)

    x_t,y_t,rot_angle = map(float,raw_input("Input Transfered Position and ROTATION ANGLE(x y theta)").split(' '))
    rot_angle = np.deg2rad(rot_angle)
    print(x_t,y_t,rot_angle)

    r_1 = np.array([[np.cos(rot_angle),-np.sin(rot_angle)],[np.sin(rot_angle),np.cos(rot_angle)]])
    if not file_lists:
        print("-------There are nothing in folder-------")

    txts = [file for file in file_lists if file.endswith(".txt")]

    for txt in txts:
        new_xs, new_ys = [], []
        print("current txt file is {}".format(txt))
        with open(file_path+txt,'r') as r:
            while True:
                line = r.readline()
                if not line:
                    break
                else:
                    line = line[:line.find("\n")]
                    x, y = map(float, line.split(","))
                    print("x,y = ",x,y)
                    old_x.append(x)
                    old_y.append(y)
                    x = x + x_t
                    y = y + y_t
                    if first ==1:
                        ref_x = x
                        ref_y = y
                        first += 1
                    print("x_t,y_t = ",x,y)
                    
                    coords = np.array([[x-ref_x],[y-ref_y]])
                    coords = np.dot(r_1,coords)
                    print(coords)
                    print("rot_x,rot_y = ",coords[0][0]+ref_x,coords[1][0]+ref_y)
                    print("------------------------------------")
                    new_xs.append(coords[0][0]+ref_x)
                    new_ys.append(coords[1][0]+ref_y)
        with open(file_path+txt,'w') as w:
            for i in range(len(new_xs)):
                w.write(str(new_xs[i])+","+str(new_ys[i])+"\n")
        plt.plot(new_xs,new_ys, '.r', label = "rotated")
    plt.plot(old_x,old_y,'.b',label = 'reference')
    plt.axis("equal")
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
