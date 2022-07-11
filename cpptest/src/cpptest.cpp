#include <iostream>
#include <cmath>
#include <vector>
#include <stdlib.h>
#include <typeinfo>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <control_to_serial/fromERP.h>

#include <eigen3/Eigen/Dense>


// ERP-42 Platform Parameters
#define MAX_STEER  28.0     // [deg]
#define L  1.04             // [m]
#define LR 0.52             // [m]
#define LF 0.52             // [m]
#define CR  1700.0 * 2.0    
#define CF  1600.0 * 2.0
#define M  400.0            // [kg]
#define IZ 1500             // [kg/m^2]

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
        ROS_ERROR("Something Wrong in Calculating Matrix");
        ROS_WARN("The Program will shut down...");
        exit(0);
    }
}


class linearKalmanFilter
{
private:
    bool isFirstTime;
    double dt;    

    // For ROS
    ros::Subscriber _gpsOdomSub;
    ros::Subscriber _gpsHeadingSub;
    ros::Subscriber _ERPSub;
    ros::Subscriber _imuSub;

    double gpsX, gpsY, gpsHeading;
    double imuYawRate;
    double erpSteer, erpSpeed, erpThrottle;

    double c_r1;
    double c_a;

    vehicleState erpCurrentState;

public:

    Eigen::MatrixXd prevSensorData;
    Eigen::MatrixXd K,F,B,U,Ei,H,Z,C;
    Eigen::MatrixXd P,Q,R;
    
    VectorXd X,_P,_Q,_R; //diag

    // Eigen::MatrixXd K(6,6);
    // Eigen::MatrixXd F(6,6);
    // Eigen::MatrixXd B(6,2);
    // Eigen::MatrixXd U(2,1);
    // Eigen::MatrixXd Ei(6,1);
    // Eigen::MatrixXd H(4,6);
    // Eigen::MatrixXd Z(4,1);
    // Eigen::MatrixXd C(6,4);


    linearKalmanFilter(double x, double y, double yaw, double vx, double vy, double omega, ros::NodeHandle *nh){

    isFirstTime = true;

    erpSteer = 0;
    erpSpeed = 0;
    erpThrottle = 0;

    dt = 0.01667;
    c_r1 = 0.1;
    c_a = 0.001;    

    X.setZero(6,1);
    X << x, y, yaw, vx, vy, omega;

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

    _P.setZero(6);
    _P << 0.1, 0.1, deg2rad(3.0), 0.01, 0.01, deg2rad(0.01);
    P = _P.asDiagonal();
    P = P.array().square();
    cout << " Initial P" << endl << P << endl;

    _Q.setZero(6);
    _Q << 0.2, 0.2, deg2rad(8.0), 0.1, 0.1, deg2rad(0.7);
    Q = _Q.asDiagonal();
    Q = Q.array().square();
    cout << " Initial Q" << endl << Q << endl;

    _R.setZero(4);
    _R << 0.08, 0.08, deg2rad(10.0), deg2rad(1.0);
    R = _R.asDiagonal();
    R = R.array().square();
    cout << " Initial R" << endl << R << endl;

    _gpsOdomSub = nh->subscribe("/odom", 1 , &linearKalmanFilter::gpsOdomCallback, this);
    _gpsHeadingSub = nh->subscribe("/current_yaw", 1 , &linearKalmanFilter::gpsHeadingCallback, this);
    _ERPSub = nh->subscribe("/erpData", 1 , &linearKalmanFilter::ERPCallback, this);
    _imuSub = nh->subscribe("/imu", 1 , &linearKalmanFilter::imuCallback, this);

    }

    void gpsOdomCallback(const nav_msgs::Odometry::ConstPtr& Odom){
        gpsX = Odom->pose.pose.position.x;
        gpsY = Odom->pose.pose.position.y;
        // ROS_INFO("GPS Odom callback works");
    }

    void gpsHeadingCallback(const std_msgs::Float64::ConstPtr& Heading){
        gpsHeading = Heading->data;
        // ROS_INFO("GPS Heading callback works");
    }

    void ERPCallback(const control_to_serial::fromERP::ConstPtr& ERP){
        erpSteer = ERP->steer / 71 * M_PI / 180;
        erpThrottle = (erpSpeed - ERP->speed / 36) / dt;
        erpSpeed = ERP->speed / 36;
        // ROS_INFO("ERP callback works");
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& Imu){
        imuYawRate = Imu->angular_velocity.z;
        calcSate();
        // ROS_INFO("IMU callback works");
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
        // cout << "Ffy    : " << Ffy << endl;
        // cout << "Fry    : " << Fry << endl;
        // cout << "R_x    : " << R_x << endl;
        // cout << "F_aero : " << F_aero << endl;
        // cout << "F_load : " << F_load << endl;

        double Ei_3 = (-(Ffy * sin(erpSteer)) * dt / M) - (F_load * dt) /M;
        double Ei_4 = (Fry * dt) / M + (Ffy * cos(erpSteer)) / M;
        double Ei_5 = ((Ffy * LF * cos(erpSteer) - Fry * LR) * dt / IZ);
        Ei << 0,
              0,
              0,
              Ei_3,
              Ei_4,
              Ei_5;
        // Ei << 0,
        //       0,
        //       0,
        //       (-(Ffy * sin(erpSteer)) * dt / M - F_load * dt /M),
        //       (Fry * dt / M + Ffy * cos(erpSteer) / M),
        //       ((Ffy * LF * cos(erpSteer) - Fry * LR) * dt / IZ);

        H << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0,
             0, 0, 0, 0, 0, 1;

        Z << gpsX, 
             gpsY, 
             gpsHeading, 
             imuYawRate;
        
        // Not yet
        // C << 1, 0, 0, 0,
        //      0, 1, 0, 0,
        //      0, 0, 1, 0,
        //      0, 0, 0, 0,
        //      0, 0, 0, 0,
        //      0, 0, 0, 1;

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

        // pinv = S2.completeOrthogonalDecomposition().pseudoInverse();
        K = P*H.transpose()*C.transpose()*pinv;

        X = X + K*C*(Z - H*X);
  
        P = P - K*C*H*P;


        prevSensorData = Z;
        double finish = ros::Time::now().toSec();
        // ROS_INFO("Calculated Yaw : %f", double(X(2,0))* 180.0 /M_PI);
        ROS_INFO("Hz : %f", 1/(finish-begin));
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
    ros::NodeHandle nh;
    cout << "whats wrong??..." << endl;
    MatrixXd mat(3,3);
    // mat = MatrixXd::
    cout << mat << endl;
    double m = 1;
    cout << m * cos(m) << endl;
    cout << "x : " << deg2rad(30) << endl;
    linearKalmanFilter lkf = linearKalmanFilter(346297.8422, 4069642.2869,-45.0 * M_PI/180.0, 0.01, 0.0, 0.0, &nh);               
    
    ros::spin();

    return 0;
}