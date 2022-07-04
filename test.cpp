#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64,h>
#include <nav_msgs/Odometry.h>
#include <control_to_serial/fromERP.h>

#include <Eigen/Dense>


// ERP-42 Platform Parameters
#define MAX_STEER  28.0  // [deg]
#define L  1.04 
#define LR 0.52
#define LF 0.52

#define CF  1600.0 * 2.0
#define CR  1700.0 * 2.0
#define M  400.0
#define IZ 1500

using namespace std;

double deg2rad(double deg){
    return deg * M_PI / 180;
}

struct vehicleState{
    double x; //[rad]
    double y; //[m / s]
    double yaw;
    double vx;
    double vy;
    double omega;
};

class linearKalmanFilter
{
private:
    bool isFirstTime;
    double dt;    
    Eigen::MatrixXd prevSensorData(4,1);

    Eigen::MatrixXd X(6,1);
    Eigen::DiagonalMatrix<double,6> P; //diag
    Eigen::DiagonalMatrix<double,6> Q; //diag
    Eigen::DiagonalMatrix<double,4> R; //diag
    Eigen::MatrixXd K(6,6);
    Eigen::MatrixXd F(6,6);
    Eigen::MatrixXd B(6,2);
    Eigen::MatrixXd U(2,1);
    Eigen::MatrixXd Ei(6,1);
    Eigen::MatrixXd H(4,6);
    Eigen::MatrixXd Z(4,1);
    Eigen::MatrixXd C(6,4);

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

public:
    linearKalmanFilter(double x, double y, double yaw, double vx, double vy, double omega, ros::NodeHandle *nh){

    _gpsOdomSub = nh.subscriber('/odom', 1 , &lineaKalmanFilter::gpsOdomCallback, this);
    _gpsHeadingSub = nh.subscriber('/current_yaw', 1 , &lineaKalmanFilter::gpsHeadingCallback, this);
    _ERPSub = nh.subscriber('/erpData', 1 , &lineaKalmanFilter::ERPCallback, this);
    _imuSub = nh.subscriber('/imu', 1 , &lineaKalmanFilter::imuCallback, this);

    isFirstTime = true;

    erpSteer = 0;
    erpSpeed = 0;
    erpThrottle = 0;

    dt = 0.01667;
    c_r1 = 0.1;
    c_a = 0.001;

    X << x, y, yaw, vx, vy, omega;
    P.diagonal() << 0.1, 0.1, deg2rad(3), 0.01, 0.01, deg2rad(0.01);
    Q.diagonal() << 0.2, 0.2, deg2rad(8), 0.1, 0.1, deg2rad(0.7);
    R.diagonal() << 0.08, 0.08, deg2rad(10), deg2rad(1);

    P = P.square();
    Q = Q.square();
    R = R.square(); 
    }

    void gpsOdomCallback(const nav_msgs::Odometry ::ConstPtr &Odom){
        gpsX = Odom.pose.pose.position.x;
        gpsY = Odom.pose.pose.position.y;
    }

    void gpsHeadingCallbck(const std_msgs::Float64::Constptr &Heading){
        gpsHeading = Heading.data;
    }

    void ERPCallback(const control_to_serial::fromERP::Constptr &ERP){
        erpSteer = ERP.steer / 71 * M_PI / 180;
        erpThrottle = (erpSpeed - ERP.speed / 36) / dt;
        erpSpeed = ERP.speed / 36;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &Imu){
        imuYawRate = Imu.angular_velocity.z;
        return &linearKalmanFilter::calcSate();
    }

    void calcSate(){
        F << 1, 0, 0, cos(X(2,0)) * dt, -sin(X(2,0)) * dt, 0,
             0, 1, 0, sin(X(2,0)) * dt, cos(X(2,0)) * dt, 0,
             0, 0, 1, 0, 0, dt,
             0, 0, 0, 1, X(5,0) * dt, 0,
             0, 0, 0, -X(5,0) * dt, 1 ,0,
             0, 0, 0, 0, 0, 1;

        B << 0, 0,
             0, 0,
             0, 0,
             dt, 0,
             0, 0,
             0, 0;

        U << erpThrottle,
             erpSteer;


        double Ffy = -CF * atan2(((X(4,0) + LF * X(5,0)) / X(3,0) - erpSteer), 1.0);
        double Fry = -CR * atan2(((X(4,0) - LR * X(5,0)) / X(3,0)), 1.0);
        double R_x = c_r1 * X(3,0);
        double F_aero = c_a * X(3,0) * X(3,0);
        double F_load = F_aero + R_x;

        Ei << 0,
              0,
              0,
              (-(Ffy * sin(erpSteer)) * dt / M - F_load * dt /m),
              (Fry * dt / M + Ffy * cos(erpSteer) / M),
              ((Ffy * Lf * np.cos(delta) - Fry * Lr) * dt / Iz);
        
        H << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0,
             0, 0, 0, 0, 0, 1;

        Z << gpsX, 
             gpsY, 
             gpsHeading, 
             imuYawRate;
        
        //Not yet
        C << bool(), 0, 0, 0,
             0, bool(), 0, 0,
             0, 0, bool(), 0,
             0, 0, 0, 0,
             0, 0, 0, 0,
             0, 0, 0, bool();

        if(isFirstTime)
        {
            isFirstTime = false;
            continue;
        }
        else
        {
            X = F * X + B * U + Ei;
            P = F * P * F.transpose() + Q;
        }
        

        Eigen::MatrixXd S1(6,6) = (H * P * H.transpose()) + R;
        Eigen::MatrixXd S2(6,6) = C * S * C.transpose();
        Eigen::MatrixXd pinv(6,6) = S2.completeOrthogonalDecomposition().pseudoInverse();
        K = P * H.transpose() * C.transpose() * pinv;
        
        X = X + K * C * (Z - H * X);
        
        P = P - K * C * H * P;


        prevSensorData = Z;
    }

    Eigen::MatrixXd getStates(){
        return X
    }
};





int main(int argc){

    ros::init(argc, argv, "cpp_test");
    ros::NodeHandle nh;

    return 0;
}