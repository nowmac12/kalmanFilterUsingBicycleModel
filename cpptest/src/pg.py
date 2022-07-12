#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

import rospy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

tempx, tempy, temph = 0.0, 0.0, 0.0
gpsX, gpsY, gpsHeading = [], [], []
kfX, kfY, kfYaw = [], [], []

def gpsOdomCallback(Odom):
	tempx = Odom.pose.pose.position.x
	tempy = Odom.pose.pose.position.y

def gpsHeadingCallback(heading):
	temph = heading.data * 180 /np.pi
	print(temph)
def kalmanCallback(kf):
	kfX.append(kf.x)
	kfY.append(kf.y)
	kfYaw.append(kf.theta * 180/ np.pi)
	gpsX.append(tempx)
	gpsY.append(tempy)
	gpsHeading.append(temph)
	

def plotter():
	plt.figure("Yaw")
	plt.plot(gpsX,gpsY,'.r', label ="gps Odom")
	plt.plot(kfX,kfY,'.b', label = "kalman Odom")
	plt.legend()
	plt.axis("equal")

	plt.figure("yaw")
	plt.plot(range(len(gpsX)),gpsHeading, '-r', label = "gps Heading")
	plt.plot(range(len(kfYaw)),kfYaw,'-b',label = "kalman Yaw")
	plt.legend()
	plt.axis("equal")
	plt.show()

def main():
	rospy.init_node("provingGround")
	rospy.loginfo("Initialized")
	gpsOdomSub = rospy.Subscriber("/odom", Odometry, gpsOdomCallback, queue_size=1)
	gpsHeadingSub = rospy.Subscriber("/current_yaw",Float64,gpsHeadingCallback ,queue_size=1)
	kalmanSub = rospy.Subscriber("/LOCAL/kalmanFiltered",Pose2D,kalmanCallback, queue_size=1)

	rospy.spin()

	plotter()



if __name__ == '__main__':
    main()