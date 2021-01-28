#pragma once
#include <cstdio>
#include "ros/ros.h"
#include "rosgraph_msgs/Clock.h"
#include <ros/service.h>
#include "gazebo_msgs/ApplyBodyWrench.h"
#include "geometry_msgs/Wrench.h"
#include <string>
//#include <gazebo/physics/physics.hh>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"


class disturbance
{

private:
    ros::NodeHandle nh_; // creating ROS NodeHandle
    ros::Subscriber sub_clk; // creating ROS subscriber
    void clk_callback(const rosgraph_msgs::Clock::ConstPtr &msg);
    bool run = false;
    ros::ServiceClient client; // ros service client to conect with gazebo service server 
    std::string igor_body_name = "igor::base_link";
    geometry_msgs::Wrench igor_wrench;
    gazebo_msgs::ApplyBodyWrench srv;

    // gazebo::physics::ModelPtr model_;
    // gazebo::physics::WorldPtr world_;
    // gazebo::physics::LinkPtr link_;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tf2Listener{tfBuffer};
    geometry_msgs::TransformStamped transformStamped;

    geometry_msgs::Point force;
    geometry_msgs::Point moment;


public:
    disturbance(); //constructor
    ~disturbance(); // destructor
    ros::Time my_time;
    
}; //end of class