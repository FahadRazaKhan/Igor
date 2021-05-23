#include "igor_l1_control.h"



igor_l1_control::igor_l1_control(ros::NodeHandle* nodehandle):nh_(*nodehandle) //Constructor
{

    sub_body_imu = nh_.subscribe<sensor_msgs::Imu>("/igor/body_imu/data",1, &igor_l1_control::body_imu_callback,this);
    sub_CoG = nh_.subscribe<geometry_msgs::PointStamped>("/cog/robot",1, &igor_l1_control::CoG_callback,this);

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



    Eigen::MatrixXf Kg = -((C*Am.inverse()*Bm).inverse()); // Feedforward gain



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
    
    
     
    
    //igor_state(2) = floorf(pitch*10000)/10000;

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
 
 


    //ROS_INFO("CoG angle: %f", CoG_angle_filtered);
    
    igorState(2) = CoG_PitchAngle_filtered;
    
    

}

Eigen::VectorXf igor_l1_control::stateEstDot(Eigen::VectorXf stateEst, Eigen::VectorXf igorState, Eigen::Vector2f thetaHat, Eigen::Vector2f sigmaHat, Eigen::Vector2f adaptiveCntrl){

        float igorStateNorm = igorState.lpNorm<Eigen::Infinity>(); // Infinity Norm
        X_hat_d = Am*stateEst + Bm*(adaptiveCntrl+ thetaHat*igorStateNorm + sigmaHat);

        return X_hat_d;

} // End of State Predictor

Eigen::Vector2f igor_l1_control::thetaHatDot(Eigen::Vector2f thetaHat, Eigen::VectorXf igorState){

        X_tilda = X_hat-igorState;
        float igorStateNorm = igorState.lpNorm<Eigen::Infinity>(); // Infinity Norm
        Eigen::Vector2f y = -1*((X_tilda.transpose()*P*Bm).transpose())*igorStateNorm;
        int thetaGain = 1000;
        float thetaMax = 100;
        float epsilonTheta = 0.001;

        thetaHat_d = thetaGain*igor_l1_control::Proj(thetaHat, y, thetaMax, epsilonTheta);

        return thetaHat_d;

} // End of parameter estimator


Eigen::Vector2f igor_l1_control::sigmaHatDot(Eigen::Vector2f sigmaHat, Eigen::VectorXf igorState){

    X_tilda = X_hat-igorState;
    Eigen::Vector2f y = -1*((X_tilda.transpose()*P*Bm).transpose());

    int sigmaGain = 1000;
    float sigmaMax = 100;
    float epsilonSigma = 0.001;
    sigmaHat_d = sigmaGain*igor_l1_control::Proj(sigmaHat, y, sigmaMax, epsilonSigma);

    return sigmaHat_d;
}



Eigen::Vector2f igor_l1_control::Proj(Eigen::Vector2f theta, Eigen::Vector2f y, float thetaMax, float epsilonTheta){
    /* This Projection operator is implemented from Naira Hovakimyan's book L1 Adaptive Control Theory. For details, please
       see Appendix B of the book.
    */
    float fTheta = (theta.transpose()*theta-pow(thetaMax,2))/(epsilonTheta*pow(thetaMax,2));
    Eigen::Vector2f fThetaGradient = (2/(epsilonTheta*pow(thetaMax,2)))*theta;

    if(fTheta<0){

        projection = y;

    }
    else if(fTheta >= 0 && fThetaGradient.transpose()*y <= 0){
        projection = y;

    }

    else if(fTheta >= 0 && fThetaGradient.transpose()*y > 0){

        projection = y-((fThetaGradient/fThetaGradient.norm())*((fThetaGradient/fThetaGradient.norm()).dot(y))*fTheta);

    }

    else{
        projection(0) = 0;
        projection(1) = 0;
    }

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
ros::MultiThreadedSpinner spinner(6); // Use 6 threads for 6 callbacks in parallel
spinner.spin(); // spin() will not return until the node has been shutdown

return 0;

} // end of main