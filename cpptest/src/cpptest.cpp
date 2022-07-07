#include <iostream>
#include <cmath>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <control_to_serial/fromERP.h>

#include <eigen3/Eigen/Dense>


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
using namespace Eigen;

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
    // Eigen::MatrixXd prevSensorData;

    // Eigen::MatrixXd X(6,6);
    // Eigen::DiagonalMatrix<double,6> P; //diag
    // Eigen::DiagonalMatrix<double,6> Q; //diag
    // Eigen::DiagonalMatrix<double,4> R; //diag
    // Eigen::MatrixXd K;
    // Eigen::MatrixXd F;
    // Eigen::MatrixXd B;
    // Eigen::MatrixXd U;
    // Eigen::MatrixXd Ei;
    // Eigen::MatrixXd H;
    // Eigen::MatrixXd Z;
    // Eigen::MatrixXd C;

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


    // this->prevSensorData.Zero(4,1);
    Eigen::MatrixXd prevSensorData;
    Eigen::MatrixXd X,K,F,B,U,Ei,H,Z,C;
    VectorXd P,Q,R; //diag

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
    X = Eigen::MatrixXd::Zero(6,1);
    X << x, y, yaw, vx, vy, omega;
    
    K = Eigen::MatrixXd::Zero(6,6);
    F = Eigen::MatrixXd::Zero(6,6);
    B = Eigen::MatrixXd::Zero(6,2);
    U = Eigen::MatrixXd::Zero(2,1);
    Ei = Eigen::MatrixXd::Zero(6,1);
    H = Eigen::MatrixXd::Zero(4,6);
    Z = Eigen::MatrixXd::Zero(4,1);
    C = Eigen::MatrixXd::Zero(6,4);

    P << 0.1, 0.1, deg2rad(3), 0.01, 0.01, deg2rad(0.01);
    P = P.asDiagonal();
    P = P.array().square();

    Q << 0.2, 0.2, deg2rad(8), 0.1, 0.1, deg2rad(0.7);
    Q = Q.asDiagonal();
    Q = Q.array().square();

    R << 0.08, 0.08, deg2rad(10), deg2rad(1);
    R = R.asDiagonal();
    R = R.array().square();

    _gpsOdomSub = nh->subscribe("/odom", 1 , &linearKalmanFilter::gpsOdomCallback, this);
    _gpsHeadingSub = nh->subscribe("/current_yaw", 1 , &linearKalmanFilter::gpsHeadingCallback, this);
    _ERPSub = nh->subscribe("/erpData", 1 , &linearKalmanFilter::ERPCallback, this);
    _imuSub = nh->subscribe("/imu", 1 , &linearKalmanFilter::imuCallback, this);





    }

    void gpsOdomCallback(const nav_msgs::Odometry::ConstPtr &Odom){
        gpsX = Odom->pose.pose.position.x;
        gpsY = Odom->pose.pose.position.y;
    }

    void gpsHeadingCallback(const std_msgs::Float64::ConstPtr &Heading){
        gpsHeading = Heading->data;
    }

    void ERPCallback(const control_to_serial::fromERP::ConstPtr &ERP){
        erpSteer = ERP->steer / 71 * M_PI / 180;
        erpThrottle = (erpSpeed - ERP->speed / 36) / dt;
        erpSpeed = ERP->speed / 36;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr &Imu){
        imuYawRate = Imu->angular_velocity.z;
        return this->calcSate();
    }

    void calcSate(){
        F << 1, 0, 0, cos(float(X(2,0))) * dt, -sin(float(X(2,0))) * dt, 0,
            0, 1, 0, sin(float(X(2,0))) * dt, cos(float(X(2,0))) * dt, 0,
            0, 0, 1, 0, 0, dt,
            0, 0, 0, 1, float(X(5,0))*dt, 0,
            0, 0, 0, -float(X(5,0))*dt, 1, 0,
            0, 0, 0, 0, 0, 1;

        B << 0, 0,
             0, 0,
             0, 0,
             dt, 0,
             0, 0,
             0, 0;

        U << erpThrottle,
             erpSteer;


        double Ffy = -CF * atan2(((X(4,0) + LF * X(5,0)) / X(5,0) - erpSteer), 1.0);
        double Fry = -CR * atan2(((X(4,0) - LR * X(5,0)) / X(5,0)), 1.0);
        double R_x = c_r1 * double(X(3,0));
        double F_aero = c_a * X(3,0) * X(3,0);
        double F_load = F_aero + R_x;

        Ei << 0,
              0,
              0,
              (-(Ffy * sin(erpSteer)) * dt / M - F_load * dt /M),
              (Fry * dt / M + Ffy * cos(erpSteer) / M),
              ((Ffy * LF * cos(erpSteer) - Fry * LR) * dt / IZ);
        
        H << 1, 0, 0, 0, 0, 0,
             0, 1, 0, 0, 0, 0,
             0, 0, 1, 0, 0, 0,
             0, 0, 0, 0, 0, 1;

        Z << gpsX, 
             gpsY, 
             gpsHeading, 
             imuYawRate;
        
        // Not yet
        C << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1, 0,
             0, 0, 0, 0,
             0, 0, 0, 0,
             0, 0, 0, 1;

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
        S1 = (H * P * H.transpose()) + R;
        // Eigen::Matrix6d 
        S2 = C * S1 * C.transpose();
        // Eigen::Matrix6d 
        pinv = S2.completeOrthogonalDecomposition().pseudoInverse();
        K = P * H.transpose() * C.transpose() * pinv;
        
        X = X + K * C * (Z - H * X);
        
        P = P - K * C * H * P;


        prevSensorData = Z;
    }

    Eigen::MatrixXd getStates(){
        return X;
    }
};


// template <typename isSameMat>
Eigen::MatrixXd updateConfirmation(Eigen::MatrixXd prev, Eigen::MatrixXd current){
	
	
	
	return prev; 
}
// template <typename 
// template <typename index>
// indexAccess (index row, index col){
// 	if (value1 > value2)
// 		std::cout << "fuck";
// 	else
// 		return std::cout << "holy";
// }

int main(int argc, char **argv){

    ros::init(argc, argv, "cpp_test");
    ros::NodeHandle nh;
	Eigen::MatrixXd mat(3,3);
    mat = Eigen::MatrixXd::Zero(3,3);
    mat << 1, 2, 3,
           4, 5, 6,
           7, 8, 9;
    Eigen::Index index;
    std::cout << mat(1,2);
	// // mat.Random();
	// Eigen::MatrixXd mat2(3,3);
	// std::cout<< mat2.rows() << std::endl;
	// // mat2.Random();
	// std::cout<< (mat==mat2) << std::endl;
	// // maxValues(3,5.5);
    return 0;
}