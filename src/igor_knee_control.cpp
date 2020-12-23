#include "igor_knee_control.h"

/***  This node is used to control Igor in Gazebo simulator. It has three controllers 1. LQR, 2. Computed Troque controller, 3. Feedforward+feedback controller.
 *    Call one controller at a time.
 * 
 *    ***/



igor_knee_control::igor_knee_control(ros::NodeHandle* nodehandle):nh_(*nodehandle) //Constructor
{
    sub_body_imu = nh_.subscribe<sensor_msgs::Imu>("/igor/body_imu/data",1, &igor_knee_control::body_imu_callback,this);
    sub_odom = nh_.subscribe<nav_msgs::Odometry>("/igor/odom",1, &igor_knee_control::odom_callback,this,ros::TransportHints().tcpNoDelay());
    sub_centerOdom = nh_.subscribe<nav_msgs::Odometry>("/igor/centerOdom",1, &igor_knee_control::centerOdom_callback,this,ros::TransportHints().tcpNoDelay());
    sub_CoG = nh_.subscribe<geometry_msgs::PointStamped>("/cog/robot",1, &igor_knee_control::CoG_callback,this);
    clk_subscriber = nh_.subscribe<rosgraph_msgs::Clock>("/clock",10,&igor_knee_control::clk_callback,this);
    joint_states_subscriber = nh_.subscribe<sensor_msgs::JointState>("/igor/joint_states",1, &igor_knee_control::joint_states_callback,this);
    sub_command_velocity = nh_.subscribe<geometry_msgs::Twist>("/igor/commands/velocity",1, &igor_knee_control::command_velocity_callback,this);
    
    CoM_MapFrame_publisher = nh_.advertise<geometry_msgs::Point>( "/igor/CoM_MapFrame", 1 );
    zram_pub = nh_.advertise<geometry_msgs::Vector3>( "/igor/zramVec", 1 );
    f_pub = nh_.advertise<geometry_msgs::Vector3>( "/igor/fVec", 1 );
    Lwheel_pub = nh_.advertise<std_msgs::Float64>( "/igor/L_wheel_joint_effort_controller/command", 1 );
    Rwheel_pub = nh_.advertise<std_msgs::Float64>( "/igor/R_wheel_joint_effort_controller/command", 1 );
    Lknee_pub = nh_.advertise<std_msgs::Float64>( "/igor/L_kfe_joint_position_controller/command", 1 );
    Rknee_pub = nh_.advertise<std_msgs::Float64>( "/igor/R_kfe_joint_position_controller/command", 1 );
    Lhip_pub = nh_.advertise<std_msgs::Float64>( "/igor/L_hfe_joint_position_controller/command", 1 );
    Rhip_pub = nh_.advertise<std_msgs::Float64>( "/igor/R_hfe_joint_position_controller/command", 1 );
    plot_publisher = nh_.advertise<std_msgs::Float32MultiArray>( "/igor/plotVec", 5);
    upper_arm_pub = nh_.advertise<std_msgs::Float64>( "/igor/Upper_arm_joint_effort_controller/command", 1 );
    fore_arm_pub = nh_.advertise<std_msgs::Float64>( "/igor/Fore_arm_joint_effort_controller/command", 1 );
    client = nh_.serviceClient<std_srvs::Empty>("/gazebo/reset_world"); // service client of gazebo service
    igor_state_publisher = nh_.advertise<std_msgs::Float32MultiArray>( "/igor/igor_state", 5);

    // LQR gains
    // k_r(0,0)= k_l(0,0) = 4*(-0.7071); // Forward position gain -ve
    // k_r(0,1)= 2*(0.7071); // Yaw gain +ve
    // k_r(0,2)= k_l(0,2) = 1.2*(-16.2331); // Pitch gain -ve
    // k_r(0,3)= k_l(0,3) = (-4.8849); // Forward speed gain -ve
    // k_r(0,4)= (0.4032); // Yaw speed gain +ve
    // k_r(0,5)= k_l(0,5)= 1.5*(-3.1893); // Pitch speed gain -ve
    // k_l(0,1)= -1*k_r(0,1);
    // k_l(0,4)= -1*k_r(0,4);

    // LQR gains for ff_fb_controller
    k_r(0,0)= k_l(0,0) = (-0.7071); // Forward position gain -ve
    k_r(0,1)= (0.7071); // Yaw gain +ve
    k_r(0,2)= k_l(0,2) = (-16.2331); // Pitch gain -ve
    k_r(0,3)= k_l(0,3) = 0.65*(-4.8849); // Forward speed gain -ve
    k_r(0,4)= 0.5*(0.4032); // Yaw speed gain +ve
    k_r(0,5)= k_l(0,5)= 1.2*(-3.1893); // Pitch speed gain -ve
    k_l(0,1)= -1*k_r(0,1);
    k_l(0,4)= -1*k_r(0,4);

    // Viscous friction matrix
    V_h(0,0) = 48.4376;  
    V_h(0,1) = 0;
    V_h(0,2) = -4.9213;
    V_h(1,0) =  0; 
    V_h(1,1) =  2.2390;
    V_h(1,2) =  0;
    V_h(2,0) = -4.9213;
    V_h(2,1) =  0; 
    V_h(2,2) =  0.5000;


    // Torque selection matrix
    E_h_inv(0,0) = 0.0503;   
    E_h_inv(0,1) = 0.1605;  
    E_h_inv(0,2) = -0.0051;
    E_h_inv(1,0) = 0.0503;  
    E_h_inv(1,1) = -0.1605;  
    E_h_inv(1,2) = -0.0051;
   
    // Computed-torque controller's gain
    Kp(0,0) = Kp1;
    Kp(0,1) = 0;
    Kp(0,2) = 0;
    Kp(1,0) = 0;
    Kp(1,1) = Kp2;
    Kp(1,2) = 0;
    Kp(2,0) = 0;
    Kp(2,1) = 0;
    Kp(2,2) = Kp3;

    Kv(0,0) = Kv1;
    Kv(0,1) = 0;
    Kv(0,2) = 0;
    Kv(1,0) = 0;
    Kv(1,1) = Kv2;
    Kv(1,2) = 0;
    Kv(2,0) = 0;
    Kv(2,1) = 0;
    Kv(2,2) = Kv3;

    // Reference states
    ref_state(0) = 0; // Center Position 
    ref_state(1) = 0; // Yaw
    ref_state(2) = 0; // Pitch
    ref_state(3) = 0; // Center velocity
    ref_state(4) = 0; // yaw velocity
    ref_state(5) = 0; // Pitch velocity


    plot_vector.data.resize(12); // Resizing std::msg array
    pub_igor_state.data.resize(5);

    // Manipulator gains
    K_pos(0,0) = 15;
    K_pos(0,1) = 0;
    K_pos(1,0) = 0;
    K_pos(1,1) = 15;

    K_vel(0,0) = 3;
    K_vel(0,1) = 0;
    K_vel(1,0) = 0;
    K_vel(1,1) = 3;


    // move_target.position.x = 0.3;
    // move_target.position.y = 0;
    // move_target.position.z = 0;

    // move_target.orientation.x = 0;
    // move_target.orientation.y = 0;
    // move_target.orientation.z = 0;
    // move_target.orientation.w = 1;

    // move_group.setPoseReferenceFrame("base_link");
    // move_group.setPoseTarget(move_target); 
    
        
} // End of constructor

void igor_knee_control::command_velocity_callback(const geometry_msgs::Twist::ConstPtr &msg){
    dwa_linear_velocity = msg->linear.x;
    dwa_angular_velocity = msg->angular.z;
}

void igor_knee_control::body_imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
        

    igor_orient = msg->orientation; // a geometery_msg quaternion
    igor_angul_vel = msg->angular_velocity;
    igor_linear_accl = msg->linear_acceleration;

    pitch_vel_y = floorf(igor_angul_vel.y*10000)/10000; // round off data upto 4 decimal points
    yaw_vel_z = floorf(igor_angul_vel.z*10000)/10000;
        
    tf::quaternionMsgToTF(igor_orient, quat); // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaternion
    
    quat.normalize(); // normalize the quaternion in case it is not normalized
    
    //the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    
     
    
    //igor_state(2) = floorf(pitch*10000)/10000;

    pitchVelVector.push_back(pitch_vel_y);
    igor_state(5) = pitch_vel_filt.filter(pitchVelVector);
    
    igor_state(1) = floorf(yaw*10000)/10000;

    yawVelVector.push_back(yaw_vel_z);
    igor_state(4) = yaw_vel_filt.filter(yawVelVector);

    linearAccelVectorX.push_back(igor_linear_accl.x);
    linearAccelVectorY.push_back(igor_linear_accl.y);
    

    plot_vector.data[2] = igor_state(1);
    //plot_vector.data[0] = (linearAccelFiltX.filter(linearAccelVectorX))-0.5; // offsetting by 0.5
    plot_vector.data[1] = linearAccelFiltY.filter(linearAccelVectorY);
    //plot_vector.data[2] = igor_linear_accl.z;

    //std::cout << "IMU X Acceleration: " << plot_vector.data[0] << std::endl;
    
    pub_igor_state.data[2] = igor_state(1);
    pub_igor_state.data[4] = igor_state(4);

    
}// End of imu_callback

void igor_knee_control::clk_callback(const rosgraph_msgs::Clock::ConstPtr &msg){

    sim_time = msg->clock;

} // End of clk_callback

void igor_knee_control::centerOdom_callback(const nav_msgs::Odometry::ConstPtr &msg){

    igor_wheelbase_position = msg->pose.pose.position;


    plot_vector.data[7] = floorf(igor_wheelbase_position.x*1000)/1000;
    plot_vector.data[8] = floorf(igor_wheelbase_position.y*1000)/1000;


    
    //ROS_INFO("Wheelbase Position X: %f",  plot_vector.data[3]);
} // End of CenterOdom_callback

void igor_knee_control::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{

    igor_pose = msg->pose; // igor pose
    igor_twist = msg->twist; // igor twist
    igor_position = igor_pose.pose.position; // igor linear position
    //igor_orient = igor_pose.pose.orientation;
    igor_linear_vel = igor_twist.twist.linear; // igor linear velocity
    //igor_angul_vel = igor_twist.twist.angular; 



    igor_pos_x = igor_position.x;
    igor_pos_x = floorf(igor_pos_x*1000)/1000;
    igor_vel_x = igor_linear_vel.x;  

    igor_pos_y = (igor_position.y);
    igor_pos_y = floorf(igor_pos_y*1000)/1000;
    igor_vel_y = igor_linear_vel.y;
    
    
    trig_vec(0) = cos(floorf(yaw*1000)/1000);
    trig_vec(1) = sin(floorf(yaw*1000)/1000);
    pos_vec(0,0) = igor_pos_x;
    pos_vec(0,1) = igor_pos_y;
    vel_vec(0,0) = igor_vel_x;
    vel_vec(0,1) = igor_vel_y;

    igor_center_position = (pos_vec*trig_vec).value();
    igor_center_vel = (vel_vec*trig_vec).value();
   
    
    // Filtering of velocity
    // vel_filt_in = igor_center_vel;
    // igor_center_vel = vel_filt_out = filt1*last_vel_filt_in + filt2*last_vel_filt_out;
    // last_vel_filt_out = vel_filt_out;
    // last_vel_filt_in = vel_filt_in;

    igor_state(0) = igor_center_position;
    igor_state(3) = floorf(igor_center_vel*1000)/1000;

    //ROS_INFO("Igor Position: %f",igor_center_position);

    plot_vector.data[0] = igor_state(0);
    //plot_vector.data[6] = igor_state(3);
    
    plot_vector.data[9] = igor_pos_y;

    pub_igor_state.data[0] = igor_pos_x;
    pub_igor_state.data[1] = igor_pos_y;
    pub_igor_state.data[3] = igor_state(3);



}// End of odom_callback


void igor_knee_control::CoG_callback(const geometry_msgs::PointStamped::ConstPtr &msg)
{

    CoG_Position = msg->point;
    CoM_vec << CoG_Position.x, CoG_Position.y, CoG_Position.z;
    pitchRotation.setRPY(0,pitch,0); // Setting Pitch rotation matrix
    tf::matrixTFToEigen(pitchRotation, pitchRotEigen); // Converting tf matrix to Eigen matrix
  

    try
    { 
        leftLegTransformStamped = leftLegTfBuffer.lookupTransform("base_link", "L_wheelActuator" , ros::Time(0));
        rightLegTransformStamped = rightLegTfBuffer.lookupTransform("base_link", "R_wheelActuator" , ros::Time(0));
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s",ex.what());
    }

    leftLegTranslation << leftLegTransformStamped.transform.translation.x, leftLegTransformStamped.transform.translation.y, leftLegTransformStamped.transform.translation.z;
    rightLegTranslation << rightLegTransformStamped.transform.translation.x, rightLegTransformStamped.transform.translation.y, rightLegTransformStamped.transform.translation.z;
    // Find the mean of the two legs' translation vectors in base_link frame
    groundPoint = 0.5*(leftLegTranslation+rightLegTranslation);
    //Get the vector starting from the "ground point" and ending at the position of the current center of mass
    CoM_line = CoM_vec - groundPoint;
    // Rotate it according to the current pitch angle of Igor
    CoM_line = pitchRotEigen * CoM_line; 
    CoM_height =  CoM_line.norm();
    // Lean/Pitch angle of CoM from the wheel base 
    leanAngle = atan2(CoM_line.x(), CoM_line.z());

    //std::cout<<"Lean angle: " << std::endl << leanAngle << std::endl;
    
    /**#####################################################**/
    
    
    //*** ZRAM Part ***////
    try
    { 
        transformStamped = tfBuffer.lookupTransform("map", "base_link" , ros::Time(0)); // CoG_Position is being published in the base_link by cog_publisher
    }
    catch (tf2::TransformException &ex) 
    {
        ROS_WARN("%s",ex.what());
    }
    
    tf2::doTransform(CoG_Position, CoG_Position, transformStamped); // Transform from base_link to map
    
    CoM_pos << (CoG_Position.x), (CoG_Position.y), (CoG_Position.z); // Eigen 3d vector of CoM position in map frame
    CoM_MapFrame_publisher.publish(CoG_Position); //Publish CoM in Map frame
   

    my_data3.push_back(CoG_Position.x);
    my_data4.push_back(CoG_Position.y);
    my_data5.push_back(CoG_Position.z);

    CoM_acc_x = (f3.filter(my_data3));
    CoM_acc_y = (f4.filter(my_data4));
    CoM_acc_z = (f5.filter(my_data5));

    CoM_accl << (CoM_acc_x), (CoM_acc_y), (CoM_acc_z);


    
    f = CoM_accl - gravity_vec; // Contact forces

    alpha = (ground_level - CoG_Position.z) / f.z();


    zram = CoM_pos + (alpha*f);

    zram_vec.x = zram.x();
    zram_vec.y = zram.y();
    zram_vec.z = zram.z();

    zram_pub.publish(zram_vec);
    f.normalize();
    f_vec.x = f.x();
    f_vec.y = f.y();
    f_vec.z = f.z();

    f_pub.publish(f_vec);


    /**#####################################################**/

    
    my_data1.push_back(leanAngle);
    

    CoG_angle_filtered = f1.filter(my_data1);
 
 


    //ROS_INFO("CoG angle: %f", CoG_angle_filtered);
    
    igor_state(2) = CoG_angle_filtered;   
    //igor_state(5) = CoG_angle_vel;
 
    plot_vector.data[4] = zram.x();
    plot_vector.data[5] = zram.y();
    plot_vector.data[6] = zram.z();

    this->lqr_controller(igor_state);
    //this->CT_controller(igor_state);
    // this->ff_fb_controller();


} // End of CoG_callback

void igor_knee_control::joint_states_callback(const sensor_msgs::JointState::ConstPtr &msg)
{
    upper_arm_angle = (msg->position[7]);
    fore_arm_angle = (msg->position[0]);
    upper_arm_vel = (msg->velocity[7]);
    fore_arm_vel = (msg->velocity[0]);

    arm_angles(0) = upper_arm_angle;
    arm_angles(1) = fore_arm_angle;

    arm_angular_vel(0) = upper_arm_vel;
    arm_angular_vel(1) = fore_arm_vel;
    
    // Two link arm Jacobian
    J(0,0) = -l1*sin(upper_arm_angle)- l2*sin(upper_arm_angle + fore_arm_angle);
    J(0,1) = -l2*sin(upper_arm_angle + fore_arm_angle);
    J(1,0) = l1*cos(upper_arm_angle)+ l2*cos(upper_arm_angle + fore_arm_angle);
    J(1,1) = l2*cos(upper_arm_angle + fore_arm_angle);

    // Jacobian_dot
    J_dot(0,0) = -l1*upper_arm_vel*cos(upper_arm_angle)-l2*(upper_arm_vel+fore_arm_vel)*cos(upper_arm_angle + fore_arm_angle);
    J_dot(0,1) = -l2*(upper_arm_vel+fore_arm_vel)*cos(upper_arm_angle + fore_arm_angle);
    J_dot(1,0) = -l1*upper_arm_vel*sin(upper_arm_angle)-l2*(upper_arm_vel+fore_arm_vel)*sin(upper_arm_angle + fore_arm_angle);
    J_dot(1,1) = -l2*(upper_arm_vel+fore_arm_vel)*sin(upper_arm_angle + fore_arm_angle);

    M(0,0) = I1 + I2 + arm_m1*lg1*lg1 + arm_m2*(l1*l1 + lg2*lg2 + 2*l1*lg2*cos(fore_arm_angle));
    M(0,1) = I2 + arm_m2*(lg2*lg2+l1*lg2*cos(fore_arm_angle));
    M(1,0) = I2 + arm_m2*(lg2*lg2+l1*lg2*cos(fore_arm_angle));
    M(1,1) = I2 + arm_m2*lg2*lg2;
    
    N(0,0) = -arm_m2*l1*lg2*fore_arm_vel*(2*upper_arm_vel + fore_arm_vel)*sin(fore_arm_angle);
    N(1,0) = arm_m2*l1*lg2*upper_arm_vel*upper_arm_vel*sin(fore_arm_angle);

    EE_vel = J*arm_angular_vel; // X and Y velocities
  
    EE_pos(0) = l1*cos(upper_arm_angle) + l2*cos(upper_arm_angle + fore_arm_angle); // End-effector X position
    EE_pos(1) = l1*sin(upper_arm_angle) + l2*sin(upper_arm_angle + fore_arm_angle); // End-effector Y position

    EE_pos_err = EE_pos_ref-EE_pos;
    EE_vel_err = EE_vel_ref-EE_vel;

    armFeedb = J.colPivHouseholderQr().solve(accl_d+K_pos*EE_pos_err + K_vel*EE_vel_err - (J_dot*arm_angular_vel));

    tau = M*armFeedb+N;


    upper_arm_trq.data = tau(0);
    fore_arm_trq.data = tau(1);

    upper_arm_pub.publish(upper_arm_trq);
    fore_arm_pub.publish(fore_arm_trq);

} // End of JointStatesCallback

void igor_knee_control::lqr_controller (Eigen::VectorXf vec) //LQR State-feedback controller
{
    ROS_INFO("In LQR");
    ROS_INFO("Pitch angle: %f", igor_state(2));
    if (igor_state(2)>= -0.35 && igor_state(2) <= 0.35){
        
        igor_knee_control::ref_update();

        lqr_right_trq = lqr_trq_r.data =  (k_r*(ref_state-vec)).value(); // taking the scalar value of the eigen-matrx
      
        lqr_left_trq = lqr_trq_l.data =  (k_l*(ref_state-vec)).value();
        

        Lwheel_pub.publish(lqr_trq_l); // Publish left wheel torque
        Rwheel_pub.publish(lqr_trq_r); // Publish right wheel torque

    

       
    }
    else if (igor_state(2)<= -1.4 || igor_state(2) >= 1.4){
        lqr_right_trq = lqr_trq_r.data = 0;
        lqr_left_trq = lqr_trq_l.data = 0;
        Lwheel_pub.publish(lqr_trq_l);
        Rwheel_pub.publish(lqr_trq_r);
        
        // ROS_INFO("Reseting Model");
        // ros::Duration(0.5).sleep(); // sleep for half a second
        // client.call(srv); // Calling the service to reset robot model in gazebo
    }
    
    plot_vector.data[10] = lqr_right_trq;
    plot_vector.data[11] = lqr_left_trq;
    
    plot_publisher.publish(plot_vector);
    igor_state_publisher.publish(pub_igor_state);

    
} // End of lqr_controller

void igor_knee_control::CT_controller(Eigen::VectorXf vec) // Computed Torque controller
{
    ROS_INFO("In CT control");
    igor_knee_control::ref_update(); // calling the ref update function

    L = CoM_height; //0.5914; // CoM height

    velocities(0) = vec(3); // Center velocity
    velocities(1) = vec(4); // Yaw velocity
    velocities(2) = vec(5); // Pitch velocity

    // Inertia matrix
    M_h(0,0)= 8.55;
    M_h(0,1)= 0;
    M_h(0,2) = 7.5*L*cos(vec(2));
    M_h(1,0)= 0;
    M_h(1,1)= 7.5*pow(L,2) - pow(cos(vec(2)),2)*(7.5*pow(L,2) + 0.0246) + 0.1382;
    M_h(1,2)= 0;
    M_h(2,0) = 7.5*L*cos(vec(2));
    M_h(2,1)= 0;
    M_h(2,2)= 7.5*pow(L,2) + 0.0347;

   // Coriolis and centrifugal vector 
    H_h(0) = -7.5*L*sin(vec(2))*(pow(vec(4),2) + pow(vec(5),2));
    H_h(1) = 6.0000e-04*vec(4)*(12500*vec(5)*sin(2*vec(2))*pow(L,2) + 12500*vec(3)*sin(vec(2))*L + 41*vec(5)*sin(2*vec(2)));
    H_h(2) = -0.5000*pow(vec(4),2)*sin(2*vec(2))*(7.5000*pow(L,2) + 0.0246);

    // Gravity vector
    G_h(0) = 0;
    G_h(1) = 0;
    G_h(2) = -73.5750*L*sin(vec(2));

    // Position errors
    Ep(0) = ref_state(0)-vec(0);
    Ep(1) = ref_state(1)-vec(1);
    Ep(2) = ref_state(2)-vec(2);

    //std::cout << "Pos Errors: " << std::endl << Ep << std::endl; 
    
    // Velocity errors
    Ev(0) = ref_state(3)-vec(3);
    Ev(1) = ref_state(4)-vec(4);
    Ev(2) = ref_state(5)-vec(5);
    

    feedbck = Kv*Ev + Kp*Ep; 
    output_trq = E_h_inv*(M_h*(feedbck)+ H_h + V_h*velocities + G_h);

    
    CT_trq_r.data = output_trq(1); // Right wheel torque
    CT_trq_l.data = output_trq(0); // Left wheel torque
    
    
    // Lwheel_pub.publish(CT_trq_l);
    // Rwheel_pub.publish(CT_trq_r);
    

    // plot_vector.data[10] = output_trq(1); // Right wheel torque
    // plot_vector.data[11] = output_trq(0); // Left wheel torque
    // plot_publisher.publish(plot_vector);
    // igor_state_publisher.publish(pub_igor_state);


}// End of CT_controller

void igor_knee_control::ff_fb_controller(){
    ROS_INFO("In ff_fb control");
    this->lqr_controller(igor_state);
    this->CT_controller(igor_state);
    
    // plot_vector.data[11] = trq_l.data = output_trq(0) + lqr_left_trq;
    // plot_vector.data[10] = trq_r.data = output_trq(1) + lqr_right_trq;

    // Lwheel_pub.publish(trq_l);
    // Rwheel_pub.publish(trq_r);


    // plot_publisher.publish(plot_vector);
    // igor_state_publisher.publish(pub_igor_state);

}// End of ff_fb_controller


void igor_knee_control::ref_update()
{

    ROS_INFO("In ref_update");

    
    if (abs(plot_vector.data[0])>=2.0){




        accl_d(0) = -2*plot_vector.data[0]; // Endeffector X acceleration
        accl_d(1) = 0; // Endeffector Y acceleration
        
        //ref_state(0) = 0.5; //Forward position
        // ref_state(3) = 0; // Forward speed
        // ref_state(1) = cos(0.3*ros::Time::now().toSec()); // Yaw




    }

    else if(abs(igor_state(4))>=1.5 && abs(igor_state(4))<=3.5){


        accl_d(0) = -2*igor_state(4)*cos(M_PI/2); // Endeffector X acceleration
        accl_d(1) = -2*igor_state(4)*sin(M_PI/2); // Endeffector Y acceleration

    }

    else if (sim_time.toSec()>=10){

        // Kv(0,0) = -1.5;
        // Kv(1,1) = -30;
        // Kp(0,0) = 0.0;
        // Kp(1,1) = 0.0;

        // LQR Gains for DWA planner
        // k_r(0,0) = k_l(0,0) = 0; // Forwad position gain
        // k_r(0,1) = k_l(0,1) = 0; // Yaw gain
        // k_r(0,3)= k_l(0,3) = 4*(-4.8849); // Forward speed gain -ve
        // k_r(0,4)= 1*(0.4032); // Right wheel Yaw speed gain +ve
        // k_l(0,4)= -1*k_r(0,4); // Left wheel yaw speed gain


        // LQR Gains for FTC planner
        k_r(0,0) = k_l(0,0) = 0; // Forwad position gain
        k_r(0,1) = k_l(0,1) = 0; // Yaw gain
        k_r(0,3)= k_l(0,3) = 1.2*(-4.8849); // Forward speed gain -ve
        k_r(0,4)= 5*(0.4032); // Right wheel Yaw speed gain +ve
        k_l(0,4)= -1*k_r(0,4); // Left wheel yaw speed gain

        ref_state(3) = dwa_linear_velocity;
        ref_state(4) = dwa_angular_velocity;

      
        ROS_INFO("Linear Vel goal:: %f", ref_state(3));
        ROS_INFO("Linear Vel: %f", igor_state(3));
        ROS_INFO("Yaw Vel goal:: %f", ref_state(4));
        ROS_INFO("Yaw Vel: %f", igor_state(4));

        EE_pos_ref(0) = 0.3; // End-effector X reference
        EE_pos_ref(1) = 0.0; // End-effector Y reference
        EE_vel_ref(0) = 0; // End-effector X velocity reference
        EE_vel_ref(1) = 0; // End-effector Y velocity reference

        accl_d(0) = 0; // Endeffector X acceleration
        accl_d(1) = 0; // Endeffector Y acceleration

    
    }

    else{
        
        //Kp(0,0) = 0.0;
        //ref_state(0) = 0*(sin(0.3*ros::Time::now().toSec())); // forward position
        //ref_state(0) = igor_state(0)-0.5; // forward position
        ref_state(1) = 0.0; // yaw
        ref_state(3) = -0.4*0; // Forward speed
        
        // ROS_INFO("Forward Pos goal: %f", ref_state(0));
        // ROS_INFO("Yaw Pos goal: %f", ref_state(1));
        // ROS_INFO("Pitch goal: %f", ref_state(2));
        // ROS_INFO("Forward speed goal: %f", ref_state(3));
        // ROS_INFO("Yaw rate goal: %f", ref_state(4));
        // ROS_INFO("Pitch rate goal: %f", ref_state(5));
        
        // ROS_INFO("igor_state(3): %f", igor_state(3));


        EE_pos_ref(0) = 0.3; // End-effector X reference
        EE_pos_ref(1) = 0.0; // End-effector Y reference
        EE_vel_ref(0) = 0; // End-effector X velocity reference
        EE_vel_ref(1) = 0; // End-effector Y velocity reference

        accl_d(0) = 0; // Endeffector X acceleration
        accl_d(1) = 0; // Endeffector Y acceleration


    }
    
    //knee_ref.data = 0*2.0*abs(sin(0.3*ros::Time::now().toSec()));
    //hip_ref.data = 0*-1.0*abs(sin(0.3*ros::Time::now().toSec()));

    //plot_vector.data[1] = ref_state(0);
    plot_vector.data[3] = ref_state(1);
    //plot_vector.data[5] = ref_state(2); 
    
}// End of ref_update function

igor_knee_control::~igor_knee_control()
{
    std::cout<<"igor_knee_control object destroyed" << std::endl;
} // End of destructor


int main(int argc, char **argv){ /** The arguments to main are the means by which a system outside scope 
and understanding of your program can configure your program to do a particular task. These are command line parameters.**/


ros::init(argc, argv, "igor_controller"); // node name, can be superseded by node name in the launch file
ros::NodeHandle nh;
igor_knee_control myNode(&nh); // creating the igor_knee_control object

ros::Duration(0.1).sleep();
//ros::spin(); // only need for subscriber to call its callback function (Does not need for Publisher).
ros::MultiThreadedSpinner spinner(6); // Use 6 threads for 6 callbacks in parallel
spinner.spin(); // spin() will not return until the node has been shutdown

// ros::AsyncSpinner async_spinner(6);
// async_spinner.start();
// ros::waitForShutdown();

return 0;

} // end of main