#include "igor_l1_control.h"



igor_l1_control::igor_l1_control(ros::NodeHandle* nodehandle):nh_(*nodehandle) //Constructor
{

    sub_body_imu = nh_.subscribe<sensor_msgs::Imu>("/igor/body_imu/data",1, &igor_l1_control::body_imu_callback,this);
    sub_CoG = nh_.subscribe<geometry_msgs::PointStamped>("/cog/robot",1, &igor_l1_control::CoG_callback,this);
    sub_odom = nh_.subscribe<nav_msgs::Odometry>("/igor/odom",1, &igor_l1_control::odom_callback,this,ros::TransportHints().tcpNoDelay());

    Lwheel_pub = nh_.advertise<std_msgs::Float64>( "/igor/L_wheel_joint_effort_controller/command", 1 );
    Rwheel_pub = nh_.advertise<std_msgs::Float64>( "/igor/R_wheel_joint_effort_controller/command", 1 );
    plotPublisher = nh_.advertise<std_msgs::Float32MultiArray>( "/igor/plotingVec", 5);
    // Vector Initialization 
    X_hat(0) = 0;
    X_hat(1) = 0;
    X_hat(2) = 0;
    X_hat(3) = 0;
    X_hat(4) = 0;
    X_hat(5) = 0;

    X_hat_last(0) = 0;
    X_hat_last(1) = 0;
    X_hat_last(2) = 0;
    X_hat_last(3) = 0;
    X_hat_last(4) = 0;
    X_hat_last(5) = 0;

    X_hat_d(0) = 0;
    X_hat_d(1) = 0;
    X_hat_d(2) = 0;
    X_hat_d(3) = 0;
    X_hat_d(4) = 0;
    X_hat_d(5) = 0;

    X_hat_d_last(0) = 0;
    X_hat_d_last(1) = 0;
    X_hat_d_last(2) = 0;
    X_hat_d_last(3) = 0;
    X_hat_d_last(4) = 0;
    X_hat_d_last(5) = 0;

    X_tilda(0) = 0;
    X_tilda(1) = 0;
    X_tilda(2) = 0;
    X_tilda(3) = 0;
    X_tilda(4) = 0;
    X_tilda(5) = 0;

    igorState(0) = 0;
    igorState(1) = 0;
    igorState(2) = 0;
    igorState(3) = 0;
    igorState(4) = 0;
    igorState(5) = 0;

    // Am(0,0) = 0;
    // Am(0,1) = 0;
    // Am(0,2) = 0;
    // Am(0,3) = 1;
    // Am(0,4) = 0;
    // Am(0,5) = 0;
    // Am(1,0) = 0;
    // Am(1,1) = 0;
    // Am(1,2) = 0;
    // Am(1,3) = 0;
    // Am(1,4) = 1;
    // Am(1,5) = 0;
    // Am(2,0) = 0;
    // Am(2,1) = 0;
    // Am(2,2) = 0;
    // Am(2,3) = 0;
    // Am(2,4) = 0;
    // Am(2,5) = 1;
    // Am(3,0) = 96.2932;
    // Am(3,1) = 0;
    // Am(3,2) = -103.8463;
    // Am(3,3) = -176.3885;
    // Am(3,4) = 0;
    // Am(3,5) = -26.4754;
    // Am(4,0) = 0;
    // Am(4,1) = -179.9585;
    // Am(4,2) = 0;
    // Am(4,3) = 0;
    // Am(4,4) = -47.8622;
    // Am(4,5) = 0;
    // Am(5,0) = -189.2728;
    // Am(5,1) = 0;
    // Am(5,2) = 219.8499;
    // Am(5,3) = 539.5515;
    // Am(5,4) = 0;
    // Am(5,5) = 52.0397; 

    Bm(0,0) = 0;
    Bm(0,1) = 0;
    Bm(1,0) = 0;
    Bm(1,1) = 0;
    Bm(2,0) = 0;
    Bm(2,1) = 0;
    Bm(3,0) = 1;//9.9680;
    Bm(3,1) = 1;//9.9680;
    Bm(4,0) = 1;//18.6288;
    Bm(4,1) = -1;//-18.6288;
    Bm(5,0) = -1;//-19.5930;
    Bm(5,1) = -1;//-19.5930;   

    P(0,0) = 0.0009346;//14.6917;
    P(0,1) = 0;
    P(0,2) = 0;//-2.2093;
    P(0,3) = 0;//-0.2574;
    P(0,4) = 0;
    P(0,5) = 0;//-0.1283;
    P(1,0) = 0;
    P(1,1) = 0.0009346;//2.0234;
    P(1,2) = 0;
    P(1,3) = 0;
    P(1,4) = 0;//0.0028;
    P(1,5) = 0;
    P(2,0) = 0;//-2.2093;
    P(2,1) = 0;
    P(2,2) = 0.0009346;//2.1285;
    P(2,3) = 0;//0.1523;
    P(2,4) = 0;
    P(2,5) = 0;//0.0697;
    P(3,0) = 0;//-0.2574;
    P(3,1) = 0;
    P(3,2) = 0;//0.1523;
    P(3,3) = 0.0009346;//0.2631;
    P(3,4) = 0;
    P(3,5) = 0;//0.0856;
    P(4,0) = 0;
    P(4,1) = 0;//0.0028;
    P(4,2) = 0;
    P(4,3) = 0;
    P(4,4) = 0.0009346;//0.0105;
    P(4,5) = 0;
    P(5,0) = 0;//-0.1283;
    P(5,1) = 0;
    P(5,2) = 0;//0.0697;
    P(5,3) = 0;//0.0856;
    P(5,4) = 0;
    P(5,5) = 0.0009346;//0.0326; 

    // Reference states
    refState(0) = 0; // Center Position 
    refState(1) = 0; // Yaw
    refState(2) = 0; // Pitch
    refState(3) = 0; // Center velocity
    refState(4) = 0; // yaw velocity
    refState(5) = 0; // Pitch velocity


    Am_Inv = Am.completeOrthogonalDecomposition().pseudoInverse();
    Kg = (C*Am_Inv*Bm).completeOrthogonalDecomposition().pseudoInverse(); // Feedforward gain
    Kg = -1*Kg;

    PlotingVector.data.resize(12); // Resizing std::msg array

    k_r(0,0)= k_l(0,0) = 4*(-0.7071); // Forward position gain -ve
    k_r(0,1)= 2*(0.7071); // Yaw gain +ve
    k_r(0,2)= k_l(0,2) = 1.2*(-16.2331); // Pitch gain -ve
    k_r(0,3)= k_l(0,3) = (-4.8849); // Forward speed gain -ve
    k_r(0,4)= (0.4032); // Yaw speed gain +ve
    k_r(0,5)= k_l(0,5)= 1.5*(-3.1893); // Pitch speed gain -ve
    k_l(0,1)= -1*k_r(0,1);
    k_l(0,4)= -1*k_r(0,4);
    

 
    //BiQuad bq2(0.0001416, 0.0002832, 0.0001416, -1.966, 0.967); // Digital filter

    // FilterOut.reserve(2);
    

}// End of Constructor



void igor_l1_control::body_imu_callback(const sensor_msgs::Imu::ConstPtr &msg){

    igor_orient = msg->orientation; // a geometery_msg quaternion
    igor_angul_vel = msg->angular_velocity;
    igor_linear_accl = msg->linear_acceleration;

    pitch_vel = floorf(igor_angul_vel.y*10000)/10000; // round off data upto 4 decimal points
    yaw_vel = floorf(igor_angul_vel.z*10000)/10000;
        
    tf::quaternionMsgToTF(igor_orient, quat); // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaternion
    
    quat.normalize(); // normalize the quaternion in case it is not normalized
    
    //the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    

    pitchVelVector.push_back(pitch_vel);
   
    
    
    yawVelVector.push_back(yaw_vel);
    
    
    
    igorState(1) = floorf(yaw*10000)/10000;// Yaw angle
    igorState(4) = yaw_vel_filt.filter(yawVelVector); // Yaw velocity
    igorState(5) = pitch_vel_filt.filter(pitchVelVector); // Pitch Velocity

    



}// End of imu_callback

void igor_l1_control::CoG_callback(const geometry_msgs::PointStamped::ConstPtr &msg){

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
    

    
    leanAngleVector.push_back(leanAngle);
    

    CoG_PitchAngle_filtered = pitchFilt.filter(leanAngleVector);
 
 


    //ROS_INFO("CoG angle: %f", CoG_PitchAngle_filtered);
    // std::cout << "Kg:" << std::endl << Kg << std::endl;
    // std::cout << "Bm:" << std::endl << Bm << std::endl;
    // std::cout << "Am:" << std::endl << Am << std::endl;
    
    igorState(2) = CoG_PitchAngle_filtered;

}// End of CoG_callback


void igor_l1_control::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
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
   

    igorState(0) = igor_center_position;
    igorState(3) = floorf(igor_center_vel*1000)/1000;

    //ROS_INFO("Igor Position: %f",igor_center_position);

    this->adaptation(igorState);
    this->lqr_controller(igorState);

}// End of odom_callback

void igor_l1_control::adaptation(Eigen::VectorXf igorState_){

    X_tilda = X_hat-igorState_;
    // std::cout << "X_tilda: " << std::endl <<  X_tilda << std::endl;
    PlotingVector.data[0] = igorState_(0);
    PlotingVector.data[1] = igorState_(1);
    PlotingVector.data[2] = igorState_(2);
    PlotingVector.data[3] = igorState_(3);
    PlotingVector.data[4] = igorState_(4);
    PlotingVector.data[5] = igorState_(5);

    X_hat = this->stateEst(X_hat, igorState_, thetaHat, sigmaHat, adaptiveCntrl);
    PlotingVector.data[6] = X_hat(0);
    PlotingVector.data[7] = X_hat(1);
    PlotingVector.data[8] = X_hat(2);
    PlotingVector.data[9] = X_hat(3);
    PlotingVector.data[10] = X_hat(4);
    PlotingVector.data[11] = X_hat(5);
    // std::cout << "thetaHat:" << std::endl << thetaHat  << std::endl;
    thetaHat = this->thetaHatDot(thetaHat, igorState_, X_tilda);
    sigmaHat = this->sigmaHatDot(sigmaHat, X_tilda);


    
    adaptiveCntrl = this->controlInput(igorState_, thetaHat, sigmaHat);
    // trq_r.data = adaptiveCntrl(0);
    // trq_l.data = adaptiveCntrl(1); 
    // Lwheel_pub.publish(trq_l); // Publish left wheel torque
    // Rwheel_pub.publish(trq_r); // Publish right wheel torque
    
    //std::cout << "X_hat" << std::endl << X_hat << std::endl;
    
    // std::cout << "sigmaHat:" << std::endl << sigmaHat  << std::endl;
    // std::cout << "Control Vector:" << std::endl << adaptiveCntrl << std::endl;
    // std::cout << "Time step:" << std::endl << dt << std::endl;

    plotPublisher.publish(PlotingVector);

}// End of adaptation

Eigen::VectorXf igor_l1_control::stateEst(Eigen::VectorXf stateEst_, Eigen::VectorXf igorState_, Eigen::Vector2f thetaHat_, Eigen::Vector2f sigmaHat_, Eigen::Vector2f adaptiveCntrl_){

    //ROS_INFO("In stateEst");
    float igorStateNorm = igorState_.lpNorm<Eigen::Infinity>(); // Infinity Norm
    X_hat_d = (Am*stateEst_) + Bm*(adaptiveCntrl_+ thetaHat_*igorStateNorm + sigmaHat_);
    // Trapezoidal method Integration
    X_hat  = X_hat_last + (dt*(X_hat_d+X_hat_d_last)/2);
    X_hat_last = X_hat;
    X_hat_d_last = X_hat_d;
    return X_hat;

} // End of State Predictor

Eigen::Vector2f igor_l1_control::thetaHatDot(Eigen::Vector2f thetaHat_, Eigen::VectorXf igorState_, Eigen::VectorXf X_tilda_){

    //ROS_INFO("In thetaHatDot");
    float igorStateNorm = igorState_.lpNorm<Eigen::Infinity>(); // Infinity Norm
    Eigen::Vector2f y = ((X_tilda_.transpose()*P*Bm).transpose())*igorStateNorm;
    y = -1*y;
    int thetaGain = 100000;
    float thetaMax = 300;
    float epsilonTheta = 0.1;

    Eigen::Vector2f thetaProjection = this->Proj(thetaHat_, y, thetaMax, epsilonTheta);

    thetaHat_d = thetaGain*thetaProjection;

    // Trapezoidal method Integration
    thetaHat  = thetaHat_last + (dt*(thetaHat_d+thetaHat_d_last)/2);
    thetaHat_last = thetaHat;
    thetaHat_d_last = thetaHat_d;

    // std::cout << "state Norm:" << std::endl << igorStateNorm << std::endl;
    // std::cout << "thetaProjection: " << std::endl << thetaProjection << std::endl;
    return thetaHat;

} // End of parameter estimator


Eigen::Vector2f igor_l1_control::sigmaHatDot(Eigen::Vector2f sigmaHat_, Eigen::VectorXf X_tilda_){

    //ROS_INFO("In sigmaHatDot");
    Eigen::Vector2f y = ((X_tilda_.transpose()*P*Bm).transpose());
    y = -1*y;
    int sigmaGain = 1000;
    float sigmaMax = 10;
    float epsilonSigma = 0.1;
    sigmaHat_d = sigmaGain*(this->Proj(sigmaHat_, y, sigmaMax, epsilonSigma));

    // Trapezoidal method Integration
    sigmaHat  = sigmaHat_last + (dt*(sigmaHat_d + sigmaHat_d_last)/2);
    sigmaHat_last = sigmaHat;
    sigmaHat_d_last = sigmaHat_d;

    // std::cout << "sigma Y:" << std::endl << y << std::endl;
    // std::cout << "sigmaHat_d:" << std::endl << sigmaHat_d << std::endl;
    return sigmaHat;
} // End of Sigma estimator


Eigen::Vector2f igor_l1_control::controlInput(Eigen::VectorXf igorState_, Eigen::Vector2f thetaHat_, Eigen::Vector2f sigmaHat_){

    ROS_INFO("In controlInput");
    rg = Kg*refState;
    float igorStateNorm = igorState_.lpNorm<Eigen::Infinity>(); // Infinity Norm
    Eigen::Vector2f eita = adaptiveCntrl + (thetaHat_*igorStateNorm)+ sigmaHat_;
    filterInput = (eita-rg);
    float controlInput_1 = bq1.step(filterInput(0));
    float controlInput_2 = bq2.step(filterInput(1));
    Eigen::Vector2f controlInput;
    controlInput(0) = -6*controlInput_1;
    controlInput(1) = -6*controlInput_2;
    
    return controlInput;

 } // End of Controller


 void igor_l1_control::lqr_controller (Eigen::VectorXf vec) //LQR State-feedback controller
{
    ROS_INFO("In LQR");
    //ROS_INFO("Pitch angle: %f", igor_state(2));

    if (igorState(2)>= -0.35 && igorState(2) <= 0.35){
        
        //igor_knee_control::ref_update();

        trq_r.data =  (k_r*(refState-vec)).value(); // taking the scalar value of the eigen-matrx
      
        trq_l.data =  (k_l*(refState-vec)).value();
        

        Lwheel_pub.publish(trq_l); // Publish left wheel torque
        Rwheel_pub.publish(trq_r); // Publish right wheel torque

    

       
    }
    else if (igorState(2)<= -1.4 || igorState(2) >= 1.4){
        trq_r.data = 0;
        trq_l.data = 0;
        Lwheel_pub.publish(trq_l);
        Rwheel_pub.publish(trq_r);
        
        // ROS_INFO("Reseting Model");
        // ros::Duration(0.5).sleep(); // sleep for half a second
        // client.call(srv); // Calling the service to reset robot model in gazebo
    }

    
} // End of lqr_controller

Eigen::Vector2f igor_l1_control::Proj(Eigen::Vector2f theta_, Eigen::Vector2f y_, float thetaMax_, float epsilonTheta_){
    /* This Projection operator is implemented from Naira Hovakimyan's book L1 Adaptive Control Theory. For details, please
       see Appendix B of the book.
    */
    //ROS_INFO("In Projection Operator");

    Eigen::Vector2f projection;

    float fTheta_Num = ((1+epsilonTheta_)*(theta_.dot(theta_)))-(pow(thetaMax_,2));
    float fTheta_Den = epsilonTheta_*(pow(thetaMax_,2));
    float fTheta = fTheta_Num/fTheta_Den;
    // ROS_INFO("fTheta %f: ",fTheta);
    // std::cout << "Projection Y: " << std::endl << y_ << std::endl;
    float fthetaGradientCoeff = 2*(epsilonTheta_+1)/(epsilonTheta_*pow(thetaMax_,2));
    Eigen::Vector2f fThetaGradient = fthetaGradientCoeff*theta_;
    Eigen::Vector2f fThetaGradientTrans = fThetaGradient.transpose();
    float fThetaGradientNorm = fThetaGradient.norm();
    float fGradDotY = fThetaGradient.dot(y_);

    // std::cout << "fGradDotY:" << std::endl << fGradDotY << std::endl;
    // std::cout << "fThetaGradient:" << std::endl << fThetaGradient << std::endl;
    // std::cout << "fThetaGradientTranspose:" << std::endl << fThetaGradientTrans << std::endl;
    // std::cout << "fThetaGradientNorm:" << std::endl << fThetaGradientNorm << std::endl;

    if(fTheta<0){

        ROS_INFO("fTheta < 0 ");
        projection = y_;

    }
    else if(fTheta >= 0 && fGradDotY<= 0){
        ROS_INFO("fTheta >= 0 &&  fGradDotY<= 0");
        projection = y_;

    }

    else if(fTheta>=0 && fGradDotY>0){

        ROS_INFO("fTheta >= 0 && fGradDotY> 0");

        projection = y_-((fThetaGradient/fThetaGradientNorm)*((fThetaGradient/fThetaGradientNorm).dot(y_))*fTheta);

    }

    else{

        ROS_INFO("In else");
        projection(0) = 0;
        projection(1) = 0;
    }

    std::cout << "Projection:" << std::endl << projection << std::endl;
    return projection;
} // End of Projector





igor_l1_control::~igor_l1_control()
{
    std::cout<<"igor_l1_control object destroyed" << std::endl;
} // End of destructor

int main(int argc, char **argv){ /** The arguments to main are the means by which a system outside scope 
and understanding of your program can configure your program to do a particular task. These are command line parameters.**/


ros::init(argc, argv, "igor_l1_controller"); // node name, can be superseded by node name in the launch file
ros::NodeHandle nh;
igor_l1_control myNode(&nh); // creating the igor_l1_control object

ros::Duration(0.1).sleep();
//ros::spin(); // only need for subscriber to call its callback function (Does not need for Publisher).
ros::MultiThreadedSpinner spinner(3); // Use 6 threads for 6 callbacks in parallel
spinner.spin(); // spin() will not return until the node has been shutdown

return 0;

} // end of main