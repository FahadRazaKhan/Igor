#pragma once

#include <cstdio>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include <queue> // std::queue
#include <deque> // std::deque
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Vector3.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/Bool.h"
#include <Eigen/Dense>
#include <math.h>
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TwistWithCovariance.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Twist.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <ros/service.h>
#include <std_srvs/Empty.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>
#include "rosgraph_msgs/Clock.h"
#include <gram_savitzky_golay/gram_savitzky_golay.h> //gram_savitzky_golay lib
#include <boost/circular_buffer.hpp>
#include <boost/numeric/odeint.hpp> // ODE solving library
#include "BiQuad.h"


class igor_l1_control
{

    private:
        ros::NodeHandle nh_; // creating ROS NodeHandle
        ros::Subscriber sub_body_imu; // creating ROS subscriber
        ros::Subscriber sub_CoG; // creating ROS subscriber
        ros::Subscriber sub_odom; // creating ROS subscriber

        ros::Publisher  Lwheel_pub; // creating ROS publisher
        ros::Publisher  Rwheel_pub; // creating ROS publisher
        ros::Publisher  plotPublisher;


        void body_imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
        void CoG_callback(const geometry_msgs::PointStamped::ConstPtr &msg);
        void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void lqr_controller(Eigen::VectorXf eig_vec);


        Eigen::Vector2f Proj(Eigen::Vector2f theta_, Eigen::Vector2f y_, float thetaMax_, float epsilonTheta_);// Projection operator
        float Proj2(float theta_, float y_, float Pm_, float PBar_, float epsilon_);// Projection operator
        Eigen::VectorXf stateEst(Eigen::VectorXf stateEst_, Eigen::VectorXf igorState_, Eigen::Vector2f thetaHat_, Eigen::Vector2f sigmaHat_, Eigen::Vector2f adaptiveCntrl_);

        Eigen::Vector2f thetaHatDot(Eigen::Vector2f thetaHat_, Eigen::VectorXf igorState_, Eigen::VectorXf X_tilda_);
        Eigen::Vector2f sigmaHatDot(Eigen::Vector2f sigmaHat_, Eigen::VectorXf X_tilda_);
        //Eigen::MatrixXf omegaHatDot(Eigen::Vector2f omegaHat_, Eigen::Vector2f X_tilda_, Eigen::Vector2f adaptiveCntrl_);

        void adaptation(Eigen::VectorXf igorState_);
        Eigen::Vector2f controlInput(Eigen::VectorXf igorState_, Eigen::Vector2f thetaHat_, Eigen::Vector2f sigmaHat_);


        Eigen::VectorXf X_hat = Eigen::VectorXf(6); //X_hat
        Eigen::VectorXf X_hat_last = Eigen::VectorXf(6); //X_hat_previous
        Eigen::VectorXf X_hat_d = Eigen::VectorXf(6); //X_hat_dot
        Eigen::VectorXf X_hat_d_last = Eigen::VectorXf(6); //X_hat_dot_previous
        Eigen::VectorXf igorState = Eigen::VectorXf(6); // Real system states
        Eigen::VectorXf X_tilda = Eigen::VectorXf(6); // State error
        Eigen::Vector2f thetaHat{0,0}; // Parameter estimate
        Eigen::Vector2f thetaHat_last{0,0}; // Parameter estimate previous
        Eigen::Vector2f thetaHat_d{0,0}; // Parameter estimate rate
        Eigen::Vector2f thetaHat_d_last{0,0}; // Parameter estimate rate previous
        Eigen::Vector2f sigmaHat{0,0}; // Sigma estimate
        Eigen::Vector2f sigmaHat_last{0,0}; // Sigma estimate previous
        Eigen::Vector2f sigmaHat_d{0,0}; // Sigma estimate rate
        Eigen::Vector2f sigmaHat_d_last{0,0}; // Sigma estimate rate previous
        Eigen::MatrixXf omegaHat = Eigen::MatrixXf::Identity(2,2);
        Eigen::MatrixXf omegaHat_d = Eigen::MatrixXf::Zero(2,2);
        Eigen::Vector2f adaptiveCntrl{0,0}; // Adaptive control inputs
        Eigen::MatrixXf Am = -535*(Eigen::MatrixXf::Identity(6,6));
        Eigen::MatrixXf Am_Inv = Eigen::MatrixXf(6,6);
        Eigen::MatrixXf Bm = Eigen::MatrixXf(6,2);
        Eigen::MatrixXf C = Eigen::MatrixXf::Identity(6,6);
        Eigen::MatrixXf P = Eigen::MatrixXf(6,6);
        Eigen::Vector3d CoM_vec;
        Eigen::Matrix3d pitchRotEigen;
        Eigen::Vector3d groundPoint;
        Eigen::Vector3d rightLegTranslation;
        Eigen::Vector3d leftLegTranslation;
        Eigen::Vector3d CoM_line;
        Eigen::Vector2f trig_vec; // declaring 2X1 Eigen vector of datatype float
        Eigen::MatrixXf pos_vec = Eigen::MatrixXf(1,2);
        Eigen::MatrixXf vel_vec = Eigen::MatrixXf(1,2);
        Eigen::MatrixXf Kg;// = Eigen::MatrixXf(2,6);
        Eigen::VectorXf refState = Eigen::VectorXf(6);
        Eigen::MatrixXf rg; 
        Eigen::Vector2f filterInput{0,0};
        Eigen::MatrixXf k_r = Eigen::MatrixXf(1,6); // declaring 1X6 Eigen matrix of datatype float
        Eigen::MatrixXf k_l = Eigen::MatrixXf(1,6); // declaring 1X6 Eigen matrix of datatype float

        std::vector<decltype(thetaHat)::value_type> FilterOut;
       

        geometry_msgs::Quaternion  igor_orient;  // Quaternion type variable
        geometry_msgs::Vector3 igor_angul_vel; // Vector3 type variable
        geometry_msgs::Vector3 igor_linear_accl;
        geometry_msgs::Point CoG_Position;
        geometry_msgs::TransformStamped leftLegTransformStamped;
        geometry_msgs::TransformStamped rightLegTransformStamped;
        geometry_msgs::PoseWithCovariance igor_pose;
        geometry_msgs::TwistWithCovariance igor_twist;
        geometry_msgs::Point igor_position;
        geometry_msgs::Vector3 igor_linear_vel; 
        

        std_msgs::Float64 trq_r;
        std_msgs::Float64 trq_l;
        std_msgs::Float32MultiArray PlotingVector;

        tf::Quaternion quat;
        tf::Matrix3x3 pitchRotation;
        
        tf2_ros::Buffer leftLegTfBuffer;
        tf2_ros::TransformListener leftLegTfListener{leftLegTfBuffer};
        tf2_ros::Buffer rightLegTfBuffer;
        tf2_ros::TransformListener rightLegTfListener{rightLegTfBuffer};


        double roll = 0; 
        double pitch = 0; 
        double yaw = 0;

        float pitch_vel = 0;
        float yaw_vel = 0;
        float leanAngle = 0; 
        float CoM_height = 0;
        float CoG_PitchAngle_filtered = 0;

        float igor_pos_x = 0;
        float igor_pos_y = 0;
        float igor_vel_x = 0;
        float igor_vel_y = 0;
        float igor_center_position = 0;
        float igor_center_vel = 0;
        float dt = 0.002;

        
        BiQuad bq1{0.0001416, 0.0002832, 0.0001416, -1.966, 0.967};
        BiQuad bq2{0.0001416, 0.0002832, 0.0001416, -1.966, 0.967};


    public:

        igor_l1_control(ros::NodeHandle* nodehandle); // constructor
        ~igor_l1_control(); // destructor

        // Window size is 2*m+1
        const unsigned int m1 = 12;
        const unsigned int m2 = 5;
        const unsigned int m3 = 50;
        // Polynomial Order
        const unsigned int n1 = 0;
        const unsigned int n2 = 1;
        const unsigned int n3 = 2;
        // Initial Point Smoothing (ie evaluate polynomial at first point in the window)
        // Points are defined in range [-m;m]
        const int t1 = m1;
        const int t2 = m2;
        const int t3 = m3;
        // Derivate? 0: no derivation, 1: first derivative...
        const unsigned int d = 0;
        //double result;
        gram_sg::SavitzkyGolayFilterConfig sg_conf1{m1,t1,n1,d,0.002}; // filter configuration
        gram_sg::SavitzkyGolayFilterConfig sg_conf2{m2,t2,n2,1,0.002}; // filter configuration
        gram_sg::SavitzkyGolayFilterConfig sg_conf3{m3,t3,n3,2,0.002}; // filter configuration
        
        gram_sg::SavitzkyGolayFilter pitchFilt{sg_conf1};
        gram_sg::SavitzkyGolayFilter pitch_vel_filt{sg_conf1};
        gram_sg::SavitzkyGolayFilter yaw_vel_filt{sg_conf1};

        boost::circular_buffer<double> pitchVelVector {boost::circular_buffer<double>((2*m1+1),0)};
        boost::circular_buffer<double> yawVelVector {boost::circular_buffer<double>((2*m1+1),0)};
        boost::circular_buffer<double> leanAngleVector {boost::circular_buffer<double>((2*m1+1),0)};



};