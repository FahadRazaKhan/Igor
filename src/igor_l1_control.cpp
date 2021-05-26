#include "igor_l1_control.h"



igor_l1_control::igor_l1_control(ros::NodeHandle* nodehandle):nh_(*nodehandle) //Constructor
{

    sub_body_imu = nh_.subscribe<sensor_msgs::Imu>("/igor/body_imu/data",1, &igor_l1_control::body_imu_callback,this);
    sub_CoG = nh_.subscribe<geometry_msgs::PointStamped>("/cog/robot",1, &igor_l1_control::CoG_callback,this);
    sub_odom = nh_.subscribe<nav_msgs::Odometry>("/igor/odom",1, &igor_l1_control::odom_callback,this,ros::TransportHints().tcpNoDelay());

    // Vector Initialization 
    X_hat(0) = 0;
    X_hat(1) = 0;
    X_hat(2) = 0;
    X_hat(3) = 0;
    X_hat(4) = 0;
    X_hat(5) = 0;

    X_hat_d(0) = 0;
    X_hat_d(1) = 0;
    X_hat_d(2) = 0;
    X_hat_d(3) = 0;
    X_hat_d(4) = 0;
    X_hat_d(5) = 0;

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

    Am(0,0) = 0;
    Am(0,1) = 0;
    Am(0,2) = 0;
    Am(0,3) = 1;
    Am(0,4) = 0;
    Am(0,5) = 0;
    Am(1,0) = 0;
    Am(1,1) = 0;
    Am(1,2) = 0;
    Am(1,3) = 0;
    Am(1,4) = 1;
    Am(1,5) = 0;
    Am(2,0) = 0;
    Am(2,1) = 0;
    Am(2,2) = 0;
    Am(2,3) = 0;
    Am(2,4) = 0;
    Am(2,5) = 1;
    Am(3,0) = 96.2932;
    Am(3,1) = 0;
    Am(3,2) = -103.8463;
    Am(3,3) = -176.3885;
    Am(3,4) = 0;
    Am(3,5) = -26.4754;
    Am(4,0) = 0;
    Am(4,1) = -179.9585;
    Am(4,2) = 0;
    Am(4,3) = 0;
    Am(4,4) = -47.8622;
    Am(4,5) = 0;
    Am(5,0) = -189.2728;
    Am(5,1) = 0;
    Am(5,2) = 219.8499;
    Am(5,3) = 539.5515;
    Am(5,4) = 0;
    Am(5,5) = 52.0397; 

    Bm(0,0) = 0;
    Bm(0,1) = 0;
    Bm(1,0) = 0;
    Bm(1,1) = 0;
    Bm(2,0) = 0;
    Bm(2,1) = 0;
    Bm(3,0) = 9.9680;
    Bm(3,1) = 9.9680;
    Bm(4,0) = 18.6288;
    Bm(4,1) = -18.6288;
    Bm(5,0) = -19.5930;
    Bm(5,1) = -19.5930;   

    P(0,0) = 5.8438;
    P(0,1) = 0;
    P(0,2) = 6.1219;
    P(0,3) = -0.5;
    P(0,4) = 0;
    P(0,5) = 0.5759;
    P(1,0) = 0;
    P(1,1) = 0.1435;
    P(1,2) = 0;
    P(1,3) = 0;
    P(1,4) = -0.5;
    P(1,5) = 0;
    P(2,0) = 6.1219;
    P(2,1) = 0;
    P(2,2) = 6.7821;
    P(2,3) = -0.5759;
    P(2,4) = 0;
    P(2,5) = -0.5;
    P(3,0) = -0.5;
    P(3,1) = 0;
    P(3,2) = -0.5759;
    P(3,3) = 0.0721;
    P(3,4) = 0;
    P(3,5) = -0.0213;
    P(4,0) = 0;
    P(4,1) = -0.5;
    P(4,2) = 0;
    P(4,3) = 0;
    P(4,4) = 1.8904;
    P(4,5) = 0;
    P(5,0) = 0.5759;
    P(5,1) = 0;
    P(5,2) = -0.5;
    P(5,3) = -0.0213;
    P(5,4) = 0;
    P(5,5) = 4.4180; 

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

    rg = Kg*refState;

    BiQuad bq1(0.0001416, 0.0002832, 0.0001416, -1.966, 0.967); // Digital filter
    BiQuad bq2(0.0001416, 0.0002832, 0.0001416, -1.966, 0.967); // Digital filter

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
    std::cout << "rg:" << std::endl << rg << std::endl;
    std::cout << "Bm:" << std::endl << Bm << std::endl;
    std::cout << "Am_Inv:" << std::endl << Am_Inv << std::endl;
    
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


}// End of odom_callback

void igor_l1_control::adaptation(Eigen::VectorXf igorState_){

    X_tilda = X_hat-igorState_;
    thetaHat_d = this->thetaHatDot(thetaHat, igorState_, X_tilda);
    sigmaHat_d = this->sigmaHatDot(sigmaHat, X_tilda);

    thetaHat += thetaHat_d*dt;
    sigmaHat += sigmaHat_d*dt;  

    X_hat = this->stateEst(X_hat, igorState_, thetaHat, sigmaHat, adaptiveCntrl);

}// End of adaptation

Eigen::VectorXf igor_l1_control::stateEst(Eigen::VectorXf stateEst_, Eigen::VectorXf igorState_, Eigen::Vector2f thetaHat_, Eigen::Vector2f sigmaHat_, Eigen::Vector2f adaptiveCntrl_){

        float igorStateNorm = igorState_.lpNorm<Eigen::Infinity>(); // Infinity Norm
        X_hat_d = Am*stateEst_ + Bm*(adaptiveCntrl_+ thetaHat_*igorStateNorm + sigmaHat_);
        X_hat += X_hat_d*dt;

        return X_hat;

} // End of State Predictor

Eigen::Vector2f igor_l1_control::thetaHatDot(Eigen::Vector2f thetaHat_, Eigen::VectorXf igorState_, Eigen::Vector2f X_tilda_){

        
        float igorStateNorm = igorState_.lpNorm<Eigen::Infinity>(); // Infinity Norm
        Eigen::Vector2f y = -1*((X_tilda_.transpose()*P*Bm).transpose())*igorStateNorm;
        int thetaGain = 1000;
        float thetaMax = 100;
        float epsilonTheta = 0.001;

        thetaHat_d = thetaGain*igor_l1_control::Proj(thetaHat_, y, thetaMax, epsilonTheta);

        return thetaHat_d;

} // End of parameter estimator


Eigen::Vector2f igor_l1_control::sigmaHatDot(Eigen::Vector2f sigmaHat_, Eigen::Vector2f X_tilda_){

   
    Eigen::Vector2f y = -1*((X_tilda_.transpose()*P*Bm).transpose());

    int sigmaGain = 1000;
    float sigmaMax = 100;
    float epsilonSigma = 0.001;
    sigmaHat_d = sigmaGain*igor_l1_control::Proj(sigmaHat_, y, sigmaMax, epsilonSigma);

    return sigmaHat_d;
} // End of Sigma estimator

// Eigen::MatrixXf igor_l1_control::omegaHatDot(Eigen::Vector2f omegaHat_, Eigen::Vector2f X_tilda_, Eigen::Vector2f adaptiveCntrl_){

//     Eigen::MatrixXf y = -1*((X_tilda_.transpose()*P*Bm).transpose()*adaptiveCntrl_.transpose());

// }



Eigen::Vector2f igor_l1_control::Proj(Eigen::Vector2f theta_, Eigen::Vector2f y_, float thetaMax_, float epsilonTheta_){
    /* This Projection operator is implemented from Naira Hovakimyan's book L1 Adaptive Control Theory. For details, please
       see Appendix B of the book.
    */
    float fTheta = (theta_.transpose()*theta_-pow(thetaMax_,2))/(epsilonTheta_*pow(thetaMax_,2));
    Eigen::Vector2f fThetaGradient = (2/(epsilonTheta_*pow(thetaMax_,2)))*theta_;

    if(fTheta<0){

        projection = y_;

    }
    else if(fTheta >= 0 && fThetaGradient.transpose()*y_ <= 0){
        projection = y_;

    }

    else if(fTheta >= 0 && fThetaGradient.transpose()*y_ > 0){

        projection = y_-((fThetaGradient/fThetaGradient.norm())*((fThetaGradient/fThetaGradient.norm()).dot(y_))*fTheta);

    }

    else{
        projection(0) = 0;
        projection(1) = 0;
    }

    return projection;
} // End of Projector


Eigen::Vector2f igor_l1_control::controlInput(Eigen::VectorXf igorState_, Eigen::Vector2f thetaHat_){

    float igorStateNorm = igorState_.lpNorm<Eigen::Infinity>(); // Infinity Norm
    Eigen::Vector2f eita = omegaHat*adaptiveCntrl + thetaHat_*igorStateNorm;

 } // End of Controller


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
ros::spin(); // only need for subscriber to call its callback function (Does not need for Publisher).
// ros::MultiThreadedSpinner spinner(6); // Use 6 threads for 6 callbacks in parallel
// spinner.spin(); // spin() will not return until the node has been shutdown

return 0;

} // end of main