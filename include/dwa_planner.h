#ifndef __DWA_PLANNER_H
#define __DWA_PLANNER_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include "std_msgs/Header.h"
#include "std_msgs/Float64.h"
#include <std_msgs/Float32MultiArray.h>

#include <Eigen/Dense>

class DWAPlanner
{
public:
    DWAPlanner(void);

    class State
    {
    public:
        State(double, double, double, double, double);

        double x;// robot position x
        double y;// robot posiiton y
        double yaw;// robot orientation yaw
        double velocity;// robot linear velocity
        double yawrate;// robot angular velocity
    private:
    };

    class Window
    {
    public:
        Window(void);
        Window(const double, const double, const double, const double);
        double min_velocity;
        double max_velocity;
        double min_yawrate;
        double max_yawrate;
    private:
    };
    void process(void);
   
   
    Window calc_dynamic_window(void);
    float calc_to_goal_cost(const std::vector<State>& traj, const Eigen::Vector3d& goal);
    float calc_speed_cost(const std::vector<State>& traj, const float target_velocity);
    float calc_obstacle_cost(const std::vector<State>& traj, const std::vector<std::vector<float> >&);
    void motion(State& state, const double velocity, const double yawrate);
    std::vector<State> dwa_planning(Window, Eigen::Vector3d, std::vector<std::vector<float> >);

    float target_state[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    float map_resolution = 0.01;
    float pose_x = 0.0;
    float pose_y = 0.0;
    float pose_yaw = 0.0;
    float v_linear = 0.0;
    float v_angular = 0.0;
    double HZ;
    std::string ROBOT_FRAME;
    double TARGET_VELOCITY;
    double MAX_VELOCITY;
    double MIN_VELOCITY;
    double MAX_YAWRATE;
    double MAX_ACCELERATION;
    double MAX_D_YAWRATE;
    double MAX_DIST;
    double VELOCITY_RESOLUTION;
    double YAWRATE_RESOLUTION;
    double ANGLE_RESOLUTION;
    double PREDICT_TIME;
    double TO_GOAL_COST_GAIN;
    double SPEED_COST_GAIN;
    double OBSTACLE_COST_GAIN;
    double DT;
    bool USE_SCAN_AS_INPUT;

    ros::NodeHandle local_nh;

};

#endif //__DWA_PLANNER_H
