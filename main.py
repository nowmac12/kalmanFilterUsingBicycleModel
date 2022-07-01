#!/usr/bin/env python3

import rospy
import numpy as np
import math
import matplotlib.pyplot as plt

from kf import linearKalmanFilter


def main():
    rospy.init_node('linearKalmanFilter')
    rospy.loginfo('Node init complete')
    
    lkf = linearKalmanFilter(x=346297.8422,y=4069642.2869,yaw=-45*np.pi/180,vx=0.01,vy=0,omega=0)
    rospy.spin()
        
    # plt.figure("Trajectory")
    # plt.plot(lkf.gpsXs, lkf.gpsYs, 'k', label="Ground Truth")
    # plt.plot(lkf.kfStates[0,:], lkf.kfStates[1, :], '.r', label="Kalman Filter Bicycle Model")

    # # for x, y, yaw in zip(lkf.kfStates[0,:], lkf.kfStates[1, :], lkf.kfStates[2, :]):
    #     # plt.arrow(x, y, 0.01+math.cos(yaw), 0.01+math.sin(yaw))

    # plt.legend()
    # # plt.show()
    # plt.figure("Yaw")
    # plt.plot(range(len(lkf.kfStates[2, :])), lkf.kfStates[2, :] *
    #          (180/math.pi), '.r', label="Kalman Filter Bicycle Model")
    # plt.plot(range(len(lkf.gpsYaws)), np.array(lkf.gpsYaws) *
    #          (180/math.pi), '.k', label="GPS Heading")
    # plt.legend()
    # plt.show()


if __name__ == '__main__':
    main()