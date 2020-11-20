#pragma once
#include <cstdio>
#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include <visualization_msgs/Marker.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>



class igor_markers
{
    private:

        ros::NodeHandle nh_; // creating ROS NodeHandle
        ros::Subscriber center_frame;
        ros::Subscriber base_frame;
        ros::Subscriber zram_sub;
        ros::Subscriber f_sub;
        ros::Subscriber sub_CoG; // creating ROS subscriber
        
        ros::Publisher  ref_marker_pub;
        ros::Publisher  support_marker_pub;
        ros::Publisher  zram_marker_pub;
        ros::Publisher  zramArrow_marker_pub;
        ros::Publisher  f_marker_pub;
        

        void ref_frame_callback(const nav_msgs::Odometry::ConstPtr &msg);
        void support_line(const nav_msgs::Odometry::ConstPtr &msg);
        void zram_callback(const geometry_msgs::Vector3::ConstPtr &msg);
        void f_callback(const geometry_msgs::Vector3::ConstPtr &msg);
        void CoG_callback(const geometry_msgs::Point::ConstPtr &msg);

        geometry_msgs::PoseWithCovariance igor_pose;
        geometry_msgs::Point igor_position;
        geometry_msgs::Vector3 zram_;
        geometry_msgs::Vector3 f_;

        visualization_msgs::Marker ref_marker;
        visualization_msgs::Marker support_line_marker;
        visualization_msgs::Marker zram_marker;
        visualization_msgs::Marker f_marker;
        visualization_msgs::Marker zramArrow_marker;

        tf2_ros::Buffer tfBufferL, tfBufferR;
        tf2_ros::TransformListener tf2ListenerL{tfBufferL}, tf2ListenerR{tfBufferR};
        geometry_msgs::TransformStamped transformStampedL, transformStampedR;
        
        // tf2_ros::Buffer tfBuffer;
        // tf2_ros::TransformListener tf2Listener{tfBuffer};
        // geometry_msgs::TransformStamped transformStamped;
        
        
        geometry_msgs::Point Lwheel_position;
        geometry_msgs::Point Rwheel_position;
        geometry_msgs::Point zram_position;
        geometry_msgs::Point CoM_position;
        // geometry_msgs::Point CoM_position_Transformed;





    public:
        igor_markers(); //constructor
        ~igor_markers(); //destructor

}; //end of class