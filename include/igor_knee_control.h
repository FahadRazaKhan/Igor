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
//#include "Iir.h" // iir filter library
#include <gram_savitzky_golay/gram_savitzky_golay.h> //gram_savitzky_golay lib
#include <boost/circular_buffer.hpp>
//#include "dwa_planner.h"
//#include <armadillo> // Linear algebra library
//#include "kalman/ekfilter.hpp" // Kalman filter library




class igor_knee_control
{

private:

    geometry_msgs::Quaternion  igor_orient;  // Quaternion type variable
    geometry_msgs::PoseWithCovariance igor_pose;
    geometry_msgs::TwistWithCovariance igor_twist;
    geometry_msgs::Point igor_position;
    geometry_msgs::Point CoG_Position;
    geometry_msgs::Vector3 igor_linear_vel;
    geometry_msgs::Vector3 igor_angul_vel; // Vector3 type variable
    geometry_msgs::Vector3 igor_linear_accl; 
    
    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::TransformStamped leftLegTransformStamped;
    geometry_msgs::TransformStamped rightLegTransformStamped;

    geometry_msgs::Vector3 zram_vec;
    geometry_msgs::Vector3 f_vec;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf2Listener{tfBuffer};

    tf2_ros::Buffer leftLegTfBuffer;
    tf2_ros::TransformListener leftLegTfListener{leftLegTfBuffer};
    tf2_ros::Buffer rightLegTfBuffer;
    tf2_ros::TransformListener rightLegTfListener{rightLegTfBuffer};
    
    
    
    
    float igor_pos_x = 0;
    float igor_pos_y = 0;
    float igor_vel_x = 0;
    float igor_vel_y = 0;
    float igor_center_position = 0;
    float igor_center_vel = 0;

    float lqr_right_trq = 0;
    float lqr_left_trq = 0;

    float upper_arm_angle = 0; 
    float fore_arm_angle = 0;
    float upper_arm_vel = 0; 
    float fore_arm_vel = 0;

    //float last_time = 0.0;
    //float current_ros_time = 0.0;
    
    
    
    //ros::Duration dt{0}; //sampling time

    double roll = 0; 
    double pitch = 0; 
    double yaw = 0.0;
    std_msgs::Float64 lqr_trq_r;
    std_msgs::Float64 lqr_trq_l;
    std_msgs::Float64 CT_trq_r;
    std_msgs::Float64 CT_trq_l;
    std_msgs::Float64 trq_r;
    std_msgs::Float64 trq_l;
    std_msgs::Float64 upper_arm_trq;
    std_msgs::Float64 fore_arm_trq;
    std_msgs::Float64 knee_ref;
    std_msgs::Float64 hip_ref;
    std_msgs::Float32MultiArray plot_vector;
    std_msgs::Float32MultiArray pub_igor_state;

    float L = 0;
    const float l1 = 0.3; // arm lengths and center of gravity
    const float l2 = 0.3;
    const float lg1 = 0.3;
    const float lg2 = 0.3;
    const float I1 = 0.018; // Arm moment of inertia
    const float I2 = 0.0339;//0.018; // Arm moment of inertia
    const float arm_m1 = 0.6; // Arm mass
    const float arm_m2 = 0.6+0.53; // Arm mass

    float CoG_angle = 0; 
    float leanAngle = 0; 
    float CoM_height = 0;

    float CoG_angle_filtered = 0;
    //float CoG_angle_vel = 0;

    float CoM_acc_x;
    float CoM_acc_y;
    float CoM_acc_z;
    float ground_level = 0.1016; // same as wheel radius
    float alpha = 0;

    float pitch_vel_y = 0;
    float yaw_vel_z = 0;

    float dwa_linear_velocity = 0.0;
    float dwa_angular_velocity = 0.0;



    
    //float filt1 = 0.02817; // LPF const.
    //float filt2 = 0.9718; // LPF const.
    
  
    //float vel_filt_out = 0;
    //float vel_filt_in = 0;
    //float last_vel_filt_out = 0.0;
    //float last_vel_filt_in = 0.0;
    
    //geometry_msgs::Point ref_origin;
    
    
    tf::Quaternion quat;

    ros::NodeHandle nh_; // creating ROS NodeHandle
    ros::Subscriber sub_body_imu; // creating ROS subscriber
    ros::Subscriber sub_odom; // creating ROS subscriber
    ros::Subscriber sub_CoG; // creating ROS subscriber
    ros::Subscriber clk_subscriber; // creating ROS subscriber
    ros::Subscriber joint_states_subscriber; // creating ROS subscriber
    ros::Subscriber sub_command_velocity;
    

    
    ros::Publisher  Lwheel_pub; // creating ROS publisher
    ros::Publisher  Rwheel_pub; // creating ROS publisher
    ros::Publisher  Lknee_pub; // creating ROS publisher
    ros::Publisher  Rknee_pub; // creating ROS publisher
    ros::Publisher  Lhip_pub; // creating ROS publisher
    ros::Publisher  Rhip_pub; // creating ROS publisher
    ros::Publisher  zram_pub; // creating ROS publisher
    ros::Publisher  f_pub; // creating ROS publisher
    ros::Publisher  plot_publisher;
    ros::Publisher  upper_arm_pub;
    ros::Publisher  fore_arm_pub;
    ros::Publisher  igor_state_publisher;
    ros::Publisher  CoM_MapFrame_publisher;
    


    void body_imu_callback(const sensor_msgs::Imu::ConstPtr &msg);
    void joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg);
    void odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
    void CoG_callback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void lqr_controller(Eigen::VectorXf eig_vec);
    void CT_controller(Eigen::VectorXf eig_vec);
    void ff_fb_controller();
    void ref_update();
    void clk_callback(const rosgraph_msgs::Clock::ConstPtr &msg);
    void command_velocity_callback(const geometry_msgs::Twist::ConstPtr &msg);

    Eigen::Vector2f trig_vec; // declaring 2X1 Eigen vector of datatype float
    Eigen::MatrixXf pos_vec = Eigen::MatrixXf(1,2);
    Eigen::MatrixXf vel_vec = Eigen::MatrixXf(1,2);
    Eigen::MatrixXf k_r = Eigen::MatrixXf(1,6); // declaring 1X6 Eigen matrix of datatype float
    Eigen::MatrixXf k_l = Eigen::MatrixXf(1,6); // declaring 1X6 Eigen matrix of datatype float
    //Eigen::MatrixXf k_k = Eigen::MatrixXf(1,8); // declaring 1X6 Eigen matrix of datatype float
    Eigen::VectorXf igor_state = Eigen::VectorXf(6); // declaring 6x1 Eigen vector of datatype float;
    Eigen::VectorXf ref_state = Eigen::VectorXf(6);
    //Eigen::Vector3d robot_center_pos;
    Eigen::Vector3d CoM_pos;
    Eigen::Vector3d CoM_accl;
    Eigen::Vector3d zram;
    Eigen::Vector3d gravity_vec{0,0,-9.81};
    Eigen::Vector3d f;
    Eigen::Vector3d rightLegTranslation;
    Eigen::Vector3d leftLegTranslation;
    Eigen::Vector3d groundPoint;
    Eigen::Vector3d CoM_vec;
    Eigen::Vector3d CoM_line;
    Eigen::Matrix3d pitchRotEigen;

    Eigen::MatrixXd M_h = Eigen::MatrixXd(3,3);
    Eigen::Vector3d H_h;
    Eigen::MatrixXd V_h = Eigen::MatrixXd(3,3);
    Eigen::Vector3d G_h;
    Eigen::MatrixXd E_h_inv = Eigen::MatrixXd(2,3);
    Eigen::Vector3d Ep;
    Eigen::Vector3d Ev;
    Eigen::Vector3d velocities;
    Eigen::MatrixXd Kp = Eigen::MatrixXd(3,3);
    Eigen::MatrixXd Kv = Eigen::MatrixXd(3,3);

    Eigen::MatrixXf J = Eigen::MatrixXf(2,2); // Jacobian matrix
    Eigen::MatrixXf J_inv = Eigen::MatrixXf(2,2); // Jacobian inverse matrix
    Eigen::MatrixXf J_dot = Eigen::MatrixXf(2,2); // Jacobian_dot matrix
    Eigen::MatrixXf M = Eigen::MatrixXf(2,2); // Inertia matrix
    Eigen::MatrixXf K_pos = Eigen::MatrixXf(2,2); // Position gains for arm manipulator
    Eigen::MatrixXf K_vel = Eigen::MatrixXf(2,2); // Velocity gains for arm manipulator
    Eigen::Vector2f N;
    Eigen::Vector2f arm_angles{0,0};
    Eigen::Vector2f arm_angular_vel{0,0};
    Eigen::Vector2f EE_vel{0,0}; // End-effector velocities
    Eigen::Vector2f EE_vel_ref{0,0}; // End-effector reference velocities
    Eigen::Vector2f EE_vel_err{0,0}; // End-effector velocities error
    Eigen::Vector2f EE_pos{0,0}; // End-effector positions
    Eigen::Vector2f EE_pos_ref{0,0}; // End-effector reference positions
    Eigen::Vector2f EE_pos_err{0,0}; // End-effector positions error
    Eigen::Vector2f armFeedb; // arm feedback
    Eigen::Vector2f tau{0,0}; // Manipulator torques
    Eigen::Vector2f accl_d{0,0}; // Desired manipulator accelerations


    // CT gains for ff_fb_controller
    float Kp1 = -7*1.3; // Linear postion gain
    float Kp2 = -50*0.5; // Yaw gain
    float Kp3 = -95*0.6;//-105; // Pitch gain
    float Kv1 = -5*0.53; // Linear velocity gain
    float Kv2 = -10*0.3; // Yaw speed gain
    float Kv3 = -20*0.65; // Pitch speed gain

    // float Kp1 = -6.3; // Linear postion gain
    // float Kp2 = -60; // Yaw gain
    // float Kp3 = -95;//-105; // Pitch gain
    // float Kv1 = -4; // Linear velocity gain
    // float Kv2 = -10; // Yaw speed gain
    // float Kv3 = -20; // Pitch speed gain
    
    Eigen::Vector3d feedbck;
    Eigen::Vector2d output_trq;

    //Eigen::MatrixXf transf_matrix = Eigen::MatrixXf(4,4);
    //tf::Vector3 CoM_tf;
    //tf::Vector3 unit_tf;
    //tf::Vector3 rot_axis{0,1,0};
    tf::Matrix3x3 pitchRotation;
    
    ros::Time sim_time;
    ros::ServiceClient client;
    std_srvs::Empty srv;

public:

    igor_knee_control(ros::NodeHandle* nodehandle); // constructor
    ~igor_knee_control(); // destructor
    

    // Window size is 2*m+1
    const int m1 = 12;
    const int m2 = 5;
    const int m3 = 50;
    // Polynomial Order
    const int n1 = 0;
    const int n2 = 1;
    const int n3 = 2;
    // Initial Point Smoothing (ie evaluate polynomial at first point in the window)
    // Points are defined in range [-m;m]
    const int t1 = m1;
    const int t2 = m2;
    const int t3 = m3;
    // Derivate? 0: no derivation, 1: first derivative...
    const int d = 0;
    //double result;
    gram_sg::SavitzkyGolayFilterConfig sg_conf1{m1,t1,n1,d,0.002}; // filter configuration
    gram_sg::SavitzkyGolayFilterConfig sg_conf2{m2,t2,n2,1,0.002}; // filter configuration
    gram_sg::SavitzkyGolayFilterConfig sg_conf3{m3,t3,n3,2,0.002}; // filter configuration
    
    gram_sg::SavitzkyGolayFilter f1{sg_conf1};
    gram_sg::SavitzkyGolayFilter f3{sg_conf3};
    gram_sg::SavitzkyGolayFilter f4{sg_conf3};
    gram_sg::SavitzkyGolayFilter f5{sg_conf3};
    gram_sg::SavitzkyGolayFilter pitch_vel_filt{sg_conf1};
    gram_sg::SavitzkyGolayFilter yaw_vel_filt{sg_conf1};
    gram_sg::SavitzkyGolayFilter linearAccelFiltX{sg_conf1};
    gram_sg::SavitzkyGolayFilter linearAccelFiltY{sg_conf1};
    
    boost::circular_buffer<double> rightTrqVector {boost::circular_buffer<double>((2*m1+1),0)}; // Initialize with 0
    boost::circular_buffer<double> leftTrqVector {boost::circular_buffer<double>((2*m1+1),0)}; // Initialize with 0
    boost::circular_buffer<double> pitchVelVector {boost::circular_buffer<double>((2*m1+1),0)};
    boost::circular_buffer<double> yawVelVector {boost::circular_buffer<double>((2*m1+1),0)};
    boost::circular_buffer<double> linearAccelVectorX {boost::circular_buffer<double>((2*m1+1),0)};
    boost::circular_buffer<double> linearAccelVectorY {boost::circular_buffer<double>((2*m1+1),0)};
    
    boost::circular_buffer<double> my_data1 {boost::circular_buffer<double>((2*m1+1),0)};
    boost::circular_buffer<double> my_data3 {boost::circular_buffer<double>((2*m3+1),-0.033)};
    boost::circular_buffer<double> my_data4 {boost::circular_buffer<double>((2*m3+1),-0.033)};
    boost::circular_buffer<double> my_data5 {boost::circular_buffer<double>((2*m3+1),-0.033)};


}; //end of class