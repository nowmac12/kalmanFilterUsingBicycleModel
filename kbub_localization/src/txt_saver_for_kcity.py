#!/usr/bin/env python
# -*- coding: utf-8 -*-
from math import *
import rospy
import string
import os, time
from nav_msgs.msg import Odometry
import sys, tty, select, termios

import random
import matplotlib.pyplot as plt

cnt_file = 1
utm_x, utm_y = 0, 0
cx, cy = [], []
pre_x, pre_y = 0, 0

# path_name = sys.argv[1]

# if len(sys.argv) != 2:
#     print("Insufficient arguments")
#     sys.exit()

# print("Start logging File path : " + file_path)
# file_path = '/home/usera/catkin_ws/src/control_to_serial/map/'+path_name+'.txt'

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def save_path(x, y, check):

    global utm_x
    global utm_y
    global cnt_file
    global pre_x, pre_y
    global fw

    if pre_x != x and pre_y != y:
        pre_x, pre_y = x, y
        fw = open('/home/'+os.getlogin()+'/catkin_ws/src/txt_saver/map/' +
                  str(cnt_file) + '.txt', 'a+')
        fw.write(str(x))
        fw.write(',')
        fw.write(str(y))
        fw.write('\n')
        fw.close()
        if check:
            print("successfully Saved as {}.txt".format(cnt_file))
            cnt_file += 1
        # rospy.loginfo("-----Creating new path file-----")


def text_callback(msg):
    global utm_x
    global utm_y

    utm_x = msg.pose.pose.position.x
    utm_y = msg.pose.pose.position.y


def plotter(cnt_file):

	#For counting files
    path = '/home/'+os.getlogin()+'/catkin_ws/src/txt_saver/map/' 
    file_list = os.listdir(path) 

    for i in range(len(file_list)):
        cx, cy = [], []

		#create random color
        random.seed(time.time())
        r = random.random()
        g = random.random()
        b = random.random()
        random_rgb=(r,g,b)

        pw = open('/home/'+os.getlogin()+'/catkin_ws/src/txt_saver/map/' +
                  str(i+1)+'.txt', 'r')
        while True:
            line = pw.readline()
            # print(line)
            if not line:
                break
            if line.find('\n'):
                line = line[:line.find('\n')]
                cxy = (line.split(','))

            cx.append(float(cxy[0]))
            cy.append(float(cxy[1]))
        pw.close()

        plt.plot(cx, cy, c=random_rgb, marker= '.' ,markersize=6)
		#start point
        if i == 0:
            plt.text(cx[0], cy[0], "Start", fontsize=13, color='r')
            plt.text(cx[int(len(cx)/2)], cy[int(len(cy))/2], "{}.txt".format(str(i+1)),fontsize=13)
		#end point
        elif i == cnt_file-1:
            plt.text(cx[-1], cy[-1], "End", fontsize=13, color='r')
            plt.text(cx[int(len(cx)/2)], cy[int(len(cy))/2], "{}.txt".format(str(i+1)),fontsize=13)
        else:
            plt.text(cx[int(len(cx)/2)], cy[int(len(cy))/2], "{}.txt".format(str(i+1)), fontsize=13)

    tm = time.localtime(time.time())
    string = time.strftime('%Y_%m_%d_ %I_%M_%S_%p', tm)
    plt.title(string)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.axis("equal")
    plt.grid(True)

    plt.savefig('/home/'+os.getlogin()+'/catkin_ws/src/txt_saver/map/'+string+'.png',dpi=300)
    plt.show()


    


def main():
    # args = rospy.myargv(argv=sys.argv)
    # print(sys.version)
    global key
    global cnt_file
    global fw

    while not rospy.is_shutdown():

        rospy.init_node('txt_saver_for_kcity', anonymous=True)
        #---------check remaining files----------#
        path = '/home/'+os.getlogin()+'/catkin_ws/src/txt_saver/map/' 
        file_list = os.listdir(path)
        if len(file_list) !=0:
            rospy.logwarn("There are something below <map> folder")
            rospy.logerr("Please delete files below <map> folder")
            return 0

        # if len(args) <2:
        #     rospy.logerr("Input Pathname behind <rosrun> command")
        #     return 0
        # print(args[1])

        rospy.Subscriber("odom", Odometry, text_callback)
        rospy.loginfo("writing x,y for UTM")
        rospy.loginfo("Press 'P' to -pause-, 'S' to -Stop program-")

        while True:
            key = getKey()
            if key == "p":
                save_path(utm_x, utm_y, True)
                continue
            elif key == "s":
                fw.close()
                print("successfully Saved as {}.txt".format(cnt_file))
                print("{} files Made".format(cnt_file))
                rospy.loginfo("---PLOTTING GRAPH---")
                plotter(cnt_file)
                rospy.loginfo("Successfully saved and exits")
                return
            else:
                save_path(utm_x, utm_y, False)

        rospy.spin()


if __name__ == '__main__':
    settings = termios.tcgetattr(sys.stdin)
    main()
