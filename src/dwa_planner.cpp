#include "dwa_planner.h"

DWAPlanner::DWAPlanner(void)
    :local_nh("~")
{
    local_nh.param("HZ", HZ, {500});
    local_nh.param("ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
    local_nh.param("TARGET_VELOCITY", TARGET_VELOCITY, {0.0});
    local_nh.param("MAX_VELOCITY", MAX_VELOCITY, {1.0});
    local_nh.param("MIN_VELOCITY", MIN_VELOCITY, {0.0});
    local_nh.param("MAX_YAWRATE", MAX_YAWRATE, {0.8});
    local_nh.param("MAX_ACCELERATION", MAX_ACCELERATION, {1.0});
    local_nh.param("MAX_D_YAWRATE", MAX_D_YAWRATE, {2.0});
    local_nh.param("MAX_DIST", MAX_DIST, {10.0});
    local_nh.param("VELOCITY_RESOLUTION", VELOCITY_RESOLUTION, {0.1});
    local_nh.param("YAWRATE_RESOLUTION", YAWRATE_RESOLUTION, {0.1});
    local_nh.param("ANGLE_RESOLUTION", ANGLE_RESOLUTION, {0.2});
    local_nh.param("PREDICT_TIME", PREDICT_TIME, {3.0});
    local_nh.param("TO_GOAL_COST_GAIN", TO_GOAL_COST_GAIN, {1.0});
    local_nh.param("SPEED_COST_GAIN", SPEED_COST_GAIN, {1.0});
    local_nh.param("OBSTACLE_COST_GAIN", OBSTACLE_COST_GAIN, {1.0});
    local_nh.param("USE_SCAN_AS_INPUT", USE_SCAN_AS_INPUT, {false});
    DT = 1.0 / HZ;


 

}

DWAPlanner::State::State(double _x, double _y, double _yaw, double _velocity, double _yawrate)
    :x(_x), y(_y), yaw(_yaw), velocity(_velocity), yawrate(_yawrate)
{
}

DWAPlanner::Window::Window(void)
    :min_velocity(0.0), max_velocity(0.0), min_yawrate(0.0), max_yawrate(0.0)
{
}

DWAPlanner::Window::Window(const double min_v, const double max_v, const double min_y, const double max_y)
    :min_velocity(min_v), max_velocity(max_v), min_yawrate(min_y), max_yawrate(max_y)
{
}


std::vector<DWAPlanner::State> DWAPlanner::dwa_planning(
        Window dynamic_window, 
        Eigen::Vector3d goal,
        std::vector<std::vector<float> > obs_list)
{
    float min_cost = 1e6;
    float min_obs_cost = min_cost;
    float min_goal_cost = min_cost;
    float min_speed_cost = min_cost;

    std::vector<std::vector<State> > trajectories;
    std::vector<State> best_traj;

    for(float v=dynamic_window.min_velocity; v<=dynamic_window.max_velocity; v+=VELOCITY_RESOLUTION){
        for(float y=dynamic_window.min_yawrate; y<=dynamic_window.max_yawrate; y+=YAWRATE_RESOLUTION){
            State state(pose_x, pose_y, pose_yaw, v_linear, v_angular);
            std::vector<State> traj;
            for(float t=0; t<=PREDICT_TIME; t+=DT){
                motion(state, v, y);
                traj.push_back(state);
                t += DT;
            }
            trajectories.push_back(traj);

            float to_goal_cost = calc_to_goal_cost(traj, goal);
            float speed_cost = calc_speed_cost(traj, TARGET_VELOCITY);
            float obstacle_cost = calc_obstacle_cost(traj, obs_list);
            float final_cost = TO_GOAL_COST_GAIN*to_goal_cost + SPEED_COST_GAIN*speed_cost + OBSTACLE_COST_GAIN*obstacle_cost;
            if(min_cost >= final_cost){
                min_goal_cost = to_goal_cost;
                min_obs_cost = obstacle_cost;
                min_speed_cost = speed_cost;
                min_cost = final_cost;
                best_traj = traj;
            }
        }
    }
 
    if(min_cost == 1e6){
        std::vector<State> traj;
        State state(pose_x, pose_y, pose_yaw, v_linear, v_angular);
        traj.push_back(state);
        best_traj = traj;
    }
    return best_traj;
}

void DWAPlanner::process(void)
{
    
       
    Window dynamic_window = calc_dynamic_window();
    Eigen::Vector3d goal(0.3, 0.3, 0.0);
    std::vector<std::vector<float> > obs_list; // positions of obstacles
    std::vector<float> obs_state = {0.1, 0};
    obs_list.push_back(obs_state);
    obs_state[0] = 0.2; 
    obs_state[1] = 0.5;
    obs_list.push_back(obs_state);       
            
    std::vector<State> best_traj = dwa_planning(dynamic_window, goal, obs_list);

            
    target_state[0] = best_traj[0].x;
    target_state[1] = best_traj[0].y;
    target_state[2] = best_traj[0].yaw;
    target_state[3] = best_traj[0].velocity;
    target_state[4] = best_traj[0].yawrate;

    

}

DWAPlanner::Window DWAPlanner::calc_dynamic_window(void)
{
    Window window(MIN_VELOCITY, MAX_VELOCITY, -MAX_YAWRATE, MAX_YAWRATE);
    window.min_velocity = std::max((v_linear - MAX_ACCELERATION*DT), MIN_VELOCITY);
    window.max_velocity = std::min((v_linear + MAX_ACCELERATION*DT), MAX_VELOCITY);
    window.min_yawrate = std::max((v_angular - MAX_D_YAWRATE*DT), -MAX_YAWRATE);
    window.max_yawrate = std::min((v_angular + MAX_D_YAWRATE*DT),  MAX_YAWRATE);
    return window;
}

float DWAPlanner::calc_to_goal_cost(const std::vector<State>& traj, const Eigen::Vector3d& goal)
{
    Eigen::Vector3d last_position(traj.back().x, traj.back().y, traj.back().yaw);
    return (last_position.segment(0, 2) - goal.segment(0, 2)).norm();
}

float DWAPlanner::calc_speed_cost(const std::vector<State>& traj, const float target_velocity)
{
    float cost = fabs(target_velocity - traj[traj.size()-1].velocity);
    return cost;
}

float DWAPlanner::calc_obstacle_cost(const std::vector<State>& traj, const std::vector<std::vector<float> >& obs_list)
{
    float cost = 0.0;
    float min_dist = 1e3;
    for(const auto& state : traj){
        for(const auto& obs : obs_list){
            float dist = sqrt((state.x - obs[0])*(state.x - obs[0]) + (state.y - obs[1])*(state.y - obs[1]));
            if(dist <= map_resolution){
                cost = 1e6;
                return cost;
            }
            min_dist = std::min(min_dist, dist);
        }
    }
    cost = 1.0 / min_dist;
    return cost;
}

void DWAPlanner::motion(State& state, const double velocity, const double yawrate)
{
    state.yaw += yawrate*DT;
    state.x += velocity*std::cos(state.yaw)*DT;
    state.y += velocity*std::sin(state.yaw)*DT;
    state.velocity = velocity;
    state.yawrate = yawrate;
}
