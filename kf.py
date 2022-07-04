# 1. Bicycle Model & Kalman Filter Paper
#   --> https://escholarship.org/content/qt3v08d6nt/qt3v08d6nt.pdf
# 2. Bicycle Model Python Github
#   --> https://github.com/Derekabc/PathTrackingBicycle
# 3. Extended Kalman Filter Open-source Code (Python Robotics)
#   --> https://github.com/AtsushiSakai/PythonRobotics/blob/master/Localization/extended_kalman_filter/extended_kalman_filter.py
# 4. What is the Linear Kalman Filter ?
#   --> https://gaussian37.github.io/ad-ose-lkf_basic/
# 5. What is the Covariance ?
#   --> https://hsm-edu.tistory.com/1266


#TODO 1. Set 'P', 'Q', 'R' Matrix  -> ok
#     2. Set 'Ffy', 'Fry', 'm', 'Iz', 'Lf', 'Lr' parameters  -> ok
#     3. Modify 'F' Matrix  -> ok
#     4. Modify 'Z' Matrix  -> ok

#TODO 1. get 'heading_using_vector.cpp' from my computer
import numpy as np
from numpy.linalg import pinv, inv
import math

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D
from control_to_serial.msg import fromERP

# ERP-42 Parameters
max_steer = np.radians(28.0)  # [rad] max steering angle
L = 1.04                      # [m] Wheel base of vehicle
Lr = L / 2.0                  # [m]
Lf = L - Lr                   # [m]  
LENGTH = 2.02                 # [m]
WIDTH = 1.16                  # [m]

Cf = 1600.0 * 2.0  # [N/rad]
Cr = 1700.0 * 2.0  # [N/rad]
  
m = 400.0  # [kg]
Iz = 1500  # [kg/m2]    # Moment of Inertia :  m*(LENGTH**2 + WIDTH**2)/12

class linearKalmanFilter():
    
    def __init__(self, x=0.0, y=0.0, yaw=0.0, vx=0.01, vy=0, omega=0.0):
        self.isFirstTime = True
        self.prevSensorData = np.zeros((4,1))

        # <Bicycle Model Parameter>
        self.dt = 0.01667 # [sec]
        self.c_r1 = 0.1
        self.c_a = 0.001

        # <Kalman Filter Parameter> 
        # -- Initialize X_0, P_0
        self.X = np.array([x, y, yaw, vx, vy, omega]).reshape(6,1)
        self.P = np.diag([0.1, 0.1, np.deg2rad(3), 0.01, 0.01, np.deg2rad(0.01)]) ** 2 
        # -- Hyper Parameter for External Disturbance
        self.Q = np.diag([0.2, 0.2, np.deg2rad(8), 0.1, 0.1, np.deg2rad(0.7)]) ** 2
        # -- Hyper Parameter for Sensor Uncertainty
        self.R = np.diag([0.08, 0.08, np.deg2rad(10), np.deg2rad(1)]) ** 2

        paramList = ('gpsX',
                     'gpsY',
                     'gpsHeading',
                     'imuYawRate',
                     'erpSteer',
                     'erpSpeed',
                     'erpThrottle')

        for paramName in paramList:
            exec('self.'+ paramName + '= 0')

        # Declare Subscribers
        self.gpsOdomSub = rospy.Subscriber('/odom',Odometry, self.gpsOdomCallback, queue_size=1)
        self.gpsHeadingSub = rospy.Subscriber('/current_yaw',Float64, self.gpsHeadingCallback, queue_size=1)
        self.ERPSub = rospy.Subscriber('/erpData',fromERP, self.ERPCallback, queue_size=1)
        self.imuSub = rospy.Subscriber('/imu', Imu, self.imuCallback)
        
        # Declare Publisher
        self.posePub = rospy.Publisher('/kalmanFilteredData',Pose2D)
    
    # <GPS X, Y callback function>
    def gpsOdomCallback(self, odom):
        self.gpsX = odom.pose.pose.position.x   # utm x [m]
        self.gpsY = odom.pose.pose.position.y   # utm y [m]
    
    # <GPS Heading callback function>
    # You need to execute command below
    # "rosrun nmea_vtg_reader heading_using_vector"
    def gpsHeadingCallback(self, heading): 
        self.gpsHeading = heading.data          # [rad]

    # <IMU sensor callback function>
    # I only use the 'Yawrate' data of IMU Sensor
    # and this function returns 'calcState()' function. Because we need the fastest update speed
    # GPS => 8Hz, ERP => 16Hz, IMU => 30~50Hz 
    def imuCallback(self, imu):
        self.imuYawRate = imu.angular_velocity.z * np.pi/180  # [rad/s]
        return self.calcState()
    
    # <ERP Data callback function>
    # You can get 'current front steering wheel angle' and 'speed of the vehicle'
    # For the Bicycle Model, we have to calculate 'acceleration' of the vehicle [erpThrottle]
    # Ref) the last page of the ERP-42 Platform Manual doc  
    def ERPCallback(self, ERP):
        self.erpSteer = ERP.steer / 71 * np.pi/180 # [rad]
        self.erpThrottle = -(self.erpSpeed - ERP.speed / 36) / self.dt # [m/s^2]
        self.erpSpeed = ERP.speed / 36 # [m/s]

    def calcState(self):
        throttle = self.erpThrottle
        delta = self.erpSteer
        dt = self.dt
        # print(f"{self.X[1]}")
        # Prediction Matrix
        F = np.array([
            [1, 0, 0, np.cos(np.float(self.X[2]))*dt, -np.sin(np.float(self.X[2]))*dt, 0],# cos(yaw_k) x dt,  sin(yaw_k) x dt
            [0, 1, 0, np.sin(np.float(self.X[2]))*dt, np.cos(np.float(self.X[2]))*dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, np.float(self.X[5])*dt, 0],              # Omega x dt
            [0, 0, 0, -np.float(self.X[5])*dt, 1, 0],             # -Omega x dt
            [0, 0, 0, 0, 0, 1]
            ])
    
        # Control Matrix
        B = np.array([
            [0, 0],
            [0, 0],
            [0, 0],
            [dt, 0],
            [0, 0],
            [0, 0]
            ])

        # Control Vector( from ERP )
        U = np.array([
            [throttle],
            [delta]
            ])
        
        # Calculate Parameters of Bicycle Model
        Ffy = -Cf * math.atan2(((np.float(self.X[4]) + Lf * np.float(self.X[5])) / np.float(self.X[3]) - delta), 1.0)
        Fry = -Cr * math.atan2((np.float(self.X[4]) - Lr * np.float(self.X[5])) / np.float(self.X[3]), 1.0)
        R_x = self.c_r1 * np.float(self.X[3])
        F_aero = self.c_a * np.float(self.X[3]) ** 2
        F_load = F_aero + R_x
                
        # External influence    
        Ei = np.array([
            [0],
            [0],
            [0],
            [-(Ffy * np.sin(delta)) * dt / m - F_load * dt / m],
            [Fry * dt / m + Ffy * np.cos(delta) / m],
            [(Ffy * Lf * np.cos(delta) - Fry * Lr) * dt / Iz]
            ])
        # Output Matrix          
        H = np.array([           #              _ x _ y _ yaw _ vx _ vy _ omega __
            [1, 0, 0, 0, 0, 0],  #        GPS x|  1   0    0    0     0     0
            [0, 1, 0, 0, 0, 0],  #        GPS y|  0   1    0    0     0     0
            [0, 0, 1, 0, 0, 0],  #  GPS heading|  0   0    1    0     0     0
            [0, 0, 0, 0, 0, 1]   # IMU Yawrate |  0   0    0    0     0     1
            ])
    
        # Sensor Input Matrix
        Z = np.array([
            [self.gpsX],
            [self.gpsY],
            [self.gpsHeading],
            [self.imuYawRate]
        ])
    
        # 'C' Matrix is the Confirmation Matrix whether sensors updated(renewal) or not.
        isUpdated = Z == self.prevSensorData
        C = np.array([                       #       _ GPS X_ GPS Y_ GPS Heading_ IMU Yawrate
            [bool(isUpdated[0]), 0, 0, 0],   # x    |   1      0           0         0
            [0, bool(isUpdated[1]), 0, 0],   # y    |   0      1           0         0
            [0, 0, bool(isUpdated[2]), 0],   # yaw  |   0      0           1         0
            [0, 0, 0, 0],                    # Vx   |   0      0           0         0
            [0, 0, 0, 0],                    # Vy   |   0      0           0         0
            [0, 0, 0, bool(isUpdated[3])]    # omega|   0      0           0         1
        ])

        # 1.----Predict----
        # Check First Run 
        if self.isFirstTime:
            P = self.P
            self.isFirstTime = False
        else:
            # Predict State
            self.X = F @ self.X + B @ U + Ei
            # Predict Covariance
            P = F @ self.P @ F.T + self.Q  
        
        # 2.----Calculate Kalman Gain----
        S = (H @ P @ H.T) + self.R
        # print(P.shape, H.T.shape, C.T.shape, pinv(C @ (H @ P @ H.T + self.R) @ C.T).shape)
        K = P @ H.T @ C.T @ pinv(C @ S @ C.T) # K = P @ H.T @ pinv(H @ P @ H.T + R)
        
        # 3.----Estimate----
        # Estimate The State with Sensors
        self.X = self.X + K @ C @ (Z - H @ self.X)   #X = X + K @ (Z - H @ X)
        # Estimate The Covariance of State
        self.P = P - K @ C @ H @ P #P = P - K @ H @ P
        
        # Store the Previous Sensor Data for check sensor updated
        self.prevSensorData = Z

        # Publish Estimated Data
        pubData = Pose2D()
        pubData.x = np.float(self.X[0])
        pubData.y = np.float(self.X[1])
        pubData.theta = np.float(self.X[2])
        self.posePub.publish(pubData)

        # Show 'Yaw' Data
        rospy.loginfo(f"Yaw : {np.float(self.X[2]) * 180 / np.pi}")

