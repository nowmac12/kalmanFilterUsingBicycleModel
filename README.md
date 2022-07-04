# Kalman Filter Using Non-Linear Bicycle Model
### System Modeling
Using Bicycle Model
### Input
1. ERP Data

          Current Front Steering Angle
 
          Current Vehicle Speed

2. Sensors (The 'Z' Matrix in code.)

         GPS

         IMU

## Output
###Estimated Vehicle State 
**[x, y, yaw, vx, vy, omega]^T**

###ROS
ROS Topic name : /kalmanFilteredData
ROS msg type : geometry_msgs/Pose2D
ROS msg info : float64 x
               float64 y
               float64 yaw

## How to execute
### 0.ERP Control Node (/dev/ttyUSB0)
```sh
rosrun control_to_serial Serial
```
###  1.GPS (/dev/ttyUSB1)
Run GPS codes
```sh
rosrun nmea_navsat_driver nmea_serial_driver
rosrun nmea_vtg_reader heading_using_vector
rosrun gps_common utm_odometry_node
```
### 2. IMU (/dev/ttyUSB2)

Run IMU code
```sh
rosrun razor_imu_9dof imu_node.py
```
> Note: **The IMU sensor must be attached to the center of the vehicle, and pay attention to the direction of the IMU sensor coordinate system.**

### 3. KalmanFilter
```sh
rosrun txt_saver main.py
```
> Note: **Before execute, Change the permission** -> "sudo chmod +x"

> Note: **Move the above two codes to the txt_saver/src folder**

## References
1. Bicycle Model & Kalman Filter Paper
   --> https://escholarship.org/content/qt3v08d6nt/qt3v08d6nt.pdf
2. Bicycle Model Python Github
   --> https://github.com/Derekabc/PathTrackingBicycle
3. Extended Kalman Filter Open-source Code (Python Robotics)
   --> https://github.com/AtsushiSakai/PythonRobotics/blob/master/Localization/extended_kalman_filter/extended_kalman_filter.py
4. What is the Linear Kalman Filter ?
   --> https://gaussian37.github.io/ad-ose-lkf_basic/
5. What is the Covariance ?
   --> https://hsm-edu.tistory.com/1266
6. How did i solve the problem of different Sensor update speed?
   --> https://www.koreascience.or.kr/article/JAKO201411560023074.pdf
   
## TODO
1. Check Sensor Data whether properly synchronized or not.
2. Check if there is the **time delay** when compared to real-time movement.
3. Change to C++ using Eigen
4. Make the roslaunch file to run it at once 
 
