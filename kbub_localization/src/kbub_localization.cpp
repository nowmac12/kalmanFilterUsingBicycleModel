/*
 * kbub_localization.cpp
 *
 *  Created on: 2022. 07. 15.
 *      Author: Chan Yeop Lee
 */
#include <iostream>
#include <fstream>
#include <sstream>

#include <unistd.h>
#include <cmath>
#include <numeric>
#include <vector>
#include <stdlib.h>
#include <typeinfo>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <control_to_serial/fromERP.h>

#include <eigen3/Eigen/Dense>

#define PASS (void)0

// ERP-42 Platform Parameters
#define MAX_STEER  28.0     // [deg]
#define L  1.04             // [m]
#define LR 0.52             // [m]
#define LF 0.52             // [m]
#define CR  1700 * 2.0      // [N / rad]
#define CF  1700 * 2.0      // [N / rad]
#define M  250              // [kg]
#define IZ 219              // [kg/m^2]

using namespace std;
using namespace Eigen;


struct vehicleState{
    double x;    // [rad]
    double y;    // [m/s]
    double yaw;  // [rad]
    double vx;   // [m/s]
    double vy;   // [m/s]
    double omega;// [rad/s]
};

double deg2rad(double deg){
    return deg * M_PI / 180;
}

double normalizingAngle(double rad){
    if (rad > M_PI) {return (rad - M_PI*2);}
    else if (rad < -M_PI) {return (rad + M_PI*2);}
    else return rad;
}

Eigen::MatrixXd updateConfirmation(Eigen::MatrixXd prev, Eigen::MatrixXd current){

    bool updatedList[4] = {0,};
    MatrixXd confMatrix(6,4); 
    // confMatrix.setZero(6,4);
    bool isUpdated = true;

    try
    {
        for(int i = 0; i < prev.rows(); i++)
        {
            for(int j = 0; j < prev.cols(); j++)
            {
                isUpdated = (prev(i,j) != current(i,j)) ? true : false;
                // cout << "prev(i,j)" << prev(i,j)<< endl << "current(i,j)" << endl << current(i,j)<< endl;
                updatedList[i] = isUpdated;
            }
        }

        confMatrix << int(updatedList[0]), 0, 0, 0,
                      0, int(updatedList[1]), 0, 0,
                      0, 0, int(updatedList[2]), 0,
                      0, 0, 0, 0,
                      0, 0, 0, 0,
                      0, 0, 0, int(updatedList[3]);

        return confMatrix;
    }

    catch(...)
    {
        ROS_ERROR("[state] : updateConfirmation(), Something Wrong in Calculating Matrix");
        ROS_WARN("[state] : updateConfirmation(), The Program will shut down...");
        exit(0);
    }
}



class linearKalmanFilter
{
private:
    bool isFirstTime;
    bool isFlushing;
    bool isUsingMap;

    vector<double> tempX, tempY;
    double dt;    

    double _x, _y, _yaw, _vx, _vy, _omega;
    string mapLocation;

    // For ROS
    ros::Subscriber _gpsOdomSub;
    ros::Subscriber _gpsHeadingSub;
    ros::Subscriber _ERPSub;
    ros::Subscriber _imuSub;
    
    ros::Publisher _pubKf;

    double gpsX, gpsY, gpsHeading;
    double imuYawRate;
    double erpSteer, erpSpeed, erpThrottle;

    double erpPeriod;
    double c_r1;
    double c_a;

    vehicleState erpCurrentState;

public:
    /* 
    <Initializing Matrix>
    K : 6 X 6   |   F : 6 X 6
    B : 6 X 2   |   U : 2 X 1
    Ei: 6 X 1   |   P : 6 X 6 (diagonal) 
    C : 6 X 4   |   Q : 6 X 6 (diagonal)
    H : 4 X 6   |   R : 4 X 4 (diagonal)
    Z : 4 X 1   |
    */
    Eigen::MatrixXd prevSensorData;
    Eigen::MatrixXd K,F,B,U,Ei,H,Z,C; 
    Eigen::MatrixXd P,Q,R;            
    
    VectorXd X;
    VectorXd _P,_Q,_R;

    linearKalmanFilter(string mapAddress){
        ROS_INFO("[constructor] : Constructing without Initial vehicle position");    
        ROS_WARN("[constructor] : You have to execute PATH_SMOOTHING before construct");
        cout << fixed;
        cout.precision(4);
        
        mapLocation = mapAddress;

        tempX.clear(); 
        tempY.clear();

        isUsingMap = true;
        isFlushing = true;
        isFirstTime = true;

        gpsX = 0.0; 
        gpsY = 0.0; 
        gpsHeading = 0.0;
        imuYawRate = 0.0;

        dt = 0.01667;
        c_r1 = 0.1;
        c_a = 0.001;  

        erpSteer = 0;
        erpSpeed = 0;
        erpThrottle = 0;
        erpPeriod = 0.0;
    }

    linearKalmanFilter(double x, double y, double yaw, double vx, double vy, double omega){
        ROS_WARN("[constructor] : Constructing with Intial vehicle position");
        ROS_WARN("[constructor] : Be sure to check valid data");    
        cout << fixed;
        cout.precision(4);    

        _x = x;
        _y = y;
        _yaw = yaw;
        _vx = vx;
        _vy = vy;
        _omega = omega;

        isUsingMap = false;
        isFlushing = false;
        isFirstTime = true;
        
        gpsX = 0.0;
        gpsY = 0.0;
        gpsHeading = 0.0;
        imuYawRate = 0.0;

        dt = 0.0;
        c_r1 = 0.1;
        c_a = 0.001;    

        erpSteer = 0;
        erpSpeed = 0;
        erpThrottle = 0;
        erpPeriod = 0.0;
    }

    void rosInit(ros::NodeHandle *nh){
        ROS_INFO("[initializer] : rosInit(), ROS Initializing..");
        _gpsOdomSub = nh->subscribe("/odom", 1 , &linearKalmanFilter::gpsOdomCallback, this);
        _gpsHeadingSub = nh->subscribe("/current_yaw", 1 , &linearKalmanFilter::gpsHeadingCallback, this);
        _ERPSub = nh->subscribe("/erpData", 1 , &linearKalmanFilter::ERPCallback, this);
        _imuSub = nh->subscribe("/imu", 1 , &linearKalmanFilter::imuCallback, this);

        _pubKf = nh->advertise<geometry_msgs::Pose2D>("/LOCAL/kalmanFiltered", 10);

        matrixInit(nh);
    }

    void flushGPS(ros::NodeHandle *nh){
        ROS_INFO("[initializer] : flushGPS(), Waiting for GPS message.. {%s}", _gpsOdomSub.getTopic().c_str());
        boost::shared_ptr<nav_msgs::Odometry const> sharedOdom;
        ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", *nh);
        ROS_INFO("[initializer] : flushGPS(), Flushing GPS Data..");

        for(int i = 0; i < 8; i++)
        {
            sharedOdom = ros::topic::waitForMessage<nav_msgs::Odometry>("/odom", *nh);
                
            gpsX = sharedOdom->pose.pose.position.x;
            gpsY = sharedOdom->pose.pose.position.y;

            cout << "x : " << gpsX << " y : " << gpsY << endl; 
        
            tempX.push_back(gpsX);
            tempY.push_back(gpsY);

            sleep(0.125);
        }
    }

    void matrixInit(ros::NodeHandle *nh){

        ROS_INFO("[initializer] : matrixInit(), Matrix Initializing..");
        X.setZero(6,1);

        if(isUsingMap)
        {
            ROS_INFO("[initializer] : matrixInit(), Call -> flushGPS()");
            flushGPS(nh);

            ROS_INFO("[initializer] : matrixInit(), Calculating Initial GPS Data..");
            vector<double> xyS = getInitialGPS(tempX, tempY);

            ROS_INFO("[initializer] : matrixInit(), Calculating Initial Vehicle Yaw..");
            double initialYaw = getInitialYaw(mapLocation);
        
            X << xyS[0], xyS[1], initialYaw, 0.01, 0.0, 0.0;

            isFlushing = false;
        }
        else
        {
            X << _x, _y, _yaw, _vx, _vy, _omega;
        }
        
        P = Eigen::MatrixXd::Zero(6,6);
        Q = Eigen::MatrixXd::Zero(6,6);
        R = Eigen::MatrixXd::Zero(4,4);

        K = Eigen::MatrixXd::Zero(6,6);
        F = Eigen::MatrixXd::Zero(6,6);
        B = Eigen::MatrixXd::Zero(6,2);
        U = Eigen::MatrixXd::Zero(2,1);
        Ei = Eigen::MatrixXd::Zero(6,1);
        H = Eigen::MatrixXd::Zero(4,6);
        Z = Eigen::MatrixXd::Zero(4,1);
        C = Eigen::MatrixXd::Zero(6,4);

        cout << "Initial X -> " << endl << X << endl;

        _P.setZero(6);
        _P << 0.5, 0.5, deg2rad(3.0), 0.01, 0.01, deg2rad(0.01);
        P = _P.asDiagonal();
        P = P.array().square();
        cout << " Initial P -> " << endl << P << endl;

        _Q.setZero(6);
        _Q << 0.1, 0.1, deg2rad(1.0), 0.01, 0.01, deg2rad(0.5);
        Q = _Q.asDiagonal();
        Q = Q.array().square();
        cout << " Initial Q -> " << endl << Q << endl;

        _R.setZero(4);
        _R << 0.05, 0.05, deg2rad(20.0), deg2rad(0.2);
        R = _R.asDiagonal();
        R = R.array().square();
        cout << " Initial R -> " << endl << R << endl;

    }

    vector<double> getInitialGPS(vector<double> x, vector<double> y){
        ROS_WARN("[initializer] : getInitialGPS(), x.size() : %ld, y.size() : %ld",x.size(),y.size());
        if(x.size()!= y.size())
        {
            ROS_ERROR("[initializer] : getInitialGPS(), the length of the arguments is different");
            exit(0);
        }
        double meanX = std::accumulate(x.begin(), x.end(), 0.0) / x.size();
        double meanY = std::accumulate(y.begin(), y.end(), 0.0) / y.size();

        vector<double> xy;
        xy.push_back(meanX); xy.push_back(meanY);
        ROS_INFO("[initializer] : getInitialGPS(), Set Initial Position as X : %f, Y: %f",meanX, meanY);

        return xy;
    }

    vector<string> split(string sentence, char Separator){
        vector<string> answer;
        stringstream ss(sentence);
        string tmp;
    
        while (getline(ss, tmp, Separator)) {
            answer.push_back(tmp);
        }
        return answer;
    }

    double getInitialYaw(string mapAdd){
        string line;
        ifstream map(mapAdd);
        double initialYaw = 0.0;

        vector<double> mapX, mapY;
        mapX.clear(); mapY.clear();

        for(int i = 0; i<5; i++)
        {
            if (map.is_open())
            {
                getline(map, line);

                vector<string> fromMap = split(line,',');
                ROS_INFO("[initializer] : getInitialYaw(), Getting line %d x :%s, y:%s",i,fromMap[0].c_str(), fromMap[1].c_str());
                mapX.push_back(stod(fromMap[0])); mapY.push_back(stod(fromMap[1]));
            }
            else
            {
                ROS_ERROR("[initializer] : getInitialYaw(), Having trouble loading the Global Map.");
                exit(0);
            }
        }

        initialYaw = atan2(mapY.back()-mapY.front(), mapX.back()-mapX.front());
        ROS_INFO("[initializer] : getInitialYaw(), Initial Vechicle Yaw is : %f", initialYaw * 180/M_PI);
        map.close();

        return initialYaw;
        
    }

    void gpsOdomCallback(const nav_msgs::Odometry::ConstPtr& Odom){
        if(isFlushing)
        {
            PASS;
        }
        else
        {
            gpsX = Odom->pose.pose.position.x;
            gpsY = Odom->pose.pose.position.y;
        }           
    }

    void gpsHeadingCallback(const std_msgs::Float64::ConstPtr& Heading){
        if(isFlushing)
        {
            PASS;
        }
        else
        {
            gpsHeading = Heading->data;
        }
    }

    void ERPCallback(const control_to_serial::fromERP::ConstPtr& ERP){
        // erpPeriod = ros::Time::now().toSec();
        if(isFlushing)
        {
            PASS;
        }
        else
        {
            // erpPeriod = ros::Time::now().toSec() - erpPeriod;
            erpSteer = ERP->steer / 71 * M_PI / 180;
            erpThrottle = (erpSpeed - (ERP->speed / 36)) / 0.055;
            erpSpeed = ERP->speed / 36;
        }
        // double erpFinish = ros::Time::now().toSec();
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& Imu){
        if(isFlushing)
        {
            PASS;
        }
        else
        {
            imuYawRate = -(Imu->angular_velocity.z);
            calcSate();
        }
    }

    void calcSate(){
        double begin = ros::Time::now().toSec();

        F << 1, 0, 0, cos((X(2))) * dt, -sin(double(X(2))) * dt, 0,
            0, 1, 0, sin(double(X(2))) * dt, cos(double(X(2))) * dt, 0,
            0, 0, 1, 0, 0, dt,
            0, 0, 0, 1, double(X(5))*dt, 0,
            0, 0, 0, -double(X(5))*dt, 1, 0,
            0, 0, 0, 0, 0, 1;

        B << 0, 0,
             0, 0,
             0, 0,
             dt, 0,
             0, 0,
             0, 0;

        U << erpThrottle,
             erpSteer;
        
        double Ffy = -CF * atan2(((X(4) + LF * X(5)) / X(3) - erpSteer), 1.0);
        double Fry = -CR * atan2(((X(4) - LR * X(5)) / X(3)), 1.0);
        double R_x = c_r1 * double(X(3));
        double F_aero = c_a * X(3) * X(3);
        double F_load = F_aero + R_x;

        double Ei_3 = (-(Ffy * sin(erpSteer)) * dt / M) - (F_load * dt) /M;
        double Ei_4 = (Fry * dt) / M + (Ffy * cos(erpSteer)) / M;
        double Ei_5 = ((Ffy * LF * cos(erpSteer) - Fry * LR) * dt / IZ);

        Ei << 0,
              0,
              0,
              Ei_3,
              Ei_4,
              Ei_5;


        H << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0,
             0, 0, 0, 0, 0, 1;

        Z << gpsX, 
             gpsY, 
             gpsHeading, 
             imuYawRate;
        
        C = updateConfirmation(prevSensorData, Z);

        if(isFirstTime)
        {
            isFirstTime = false;
        }
        else
        {   
            X = F*X + B*U + Ei;
            P = F*P*F.transpose() + Q;
        }
        
        
        Eigen::MatrixXd S1(6,6), S2(6,6), pinv(6,6);
        S1.setZero(6,6); S2.setZero(6,6); pinv.setZero(6,6);

        S1 = (H*P*H.transpose()) + R;
        S2 = C*S1*C.transpose();
        pinv = S2.completeOrthogonalDecomposition().pseudoInverse();

        K = P*H.transpose()*C.transpose()*pinv;

        X = X + K*C*(Z - H*X);
  
        P = P - K*C*H*P;


        prevSensorData = Z;

        double finish = ros::Time::now().toSec();
        dt = finish - begin;
        
        geometry_msgs::Pose2D pose;
        pose.x = X(0);
        pose.y = X(1);
        pose.theta = X(2);
        _pubKf.publish(pose);
        // ROS_INFO("Calculated Yaw : %f", double(X(2,0))* 180.0 /M_PI);
        // ROS_INFO("time delta : %f", dt);
        ROS_INFO("[State] : calcState(), X : %f , Y : %f, Yaw : %f", X(0), X(1), normalizingAngle(X(2))* 180.0 /M_PI);
    }

    vehicleState getStates()
    {
        erpCurrentState.x = double(X(0));
        erpCurrentState.y = double(X(1));
        erpCurrentState.yaw = double(X(2));
        erpCurrentState.vx = double(X(3));
        erpCurrentState.vy = double(X(4));
        erpCurrentState.omega = double(X(5));

        return erpCurrentState;
    }
};


int main(int argc, char **argv){

    ros::init(argc, argv, "cpp_test");
    ros::NodeHandle nh("~");
    string mapAdd;

    if(nh.getParam("map",mapAdd))
    {
        ROS_INFO("[constructor] : Using Global Map to set initial position");
        linearKalmanFilter lkf = linearKalmanFilter(mapAdd);
        lkf.rosInit(&nh);

        ros::spin();
    }
    else
    {
        ROS_INFO("[constructor] : Not Using Global Map");
        ROS_WARN("[constructor] : Please Check the validation of your inputs ");
        linearKalmanFilter lkf = linearKalmanFilter(346297.3469, 4069642.8422,-45.0 * M_PI/180.0, 0.01, 0.0, 0.0);
        lkf.rosInit(&nh);
        
        ros::spin();
    }
    
    return 0;
}