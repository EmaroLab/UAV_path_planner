#include <path_planner.h>
#include <fstream>
#include <iostream>
PathPlanner::PathPlanner(){};

PathPlanner::PathPlanner(Surface_function* f1, Surface_function* f2) {
    setDefaultParameters();
    this->f1 = f1;
    this->f2 = f2;
}

void PathPlanner::setDefaultParameters() {
    hasObstacles = false;
    surfToBeDef = 1;
    surfFlag = 1;
    tangFlag = -1;
    sigma_multiplier = 15;
    xyGain = 0.15;
    zGain = 0.15;
    P_yaw = 0.7;
    D_yaw = 0.3;
    I_yaw = 0.0;
    yaw_err_i = 0;
    Kgrad1 = 0.1;
    Ktang = 0.1;
    dist_sensed_obs = 1;
    safety_margin = 0.2;
};

PathPlanner::~PathPlanner() {
};

void PathPlanner::setRobotPose(double x,double y,double z, tf::Quaternion q){
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;
    pose.header.stamp = ros::Time::now();
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.x = q.y();
    pose.pose.orientation.x = q.z();
    pose.pose.orientation.x = q.w();
    setRobotPose(pose);
};

void PathPlanner::setRobotPose(geometry_msgs::Pose pose){
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id = frame_id;
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.pose = pose;
    setRobotPose(poseStamped);
};

void PathPlanner::setRobotPose(geometry_msgs::PoseStamped pose){
    robotPose = pose;
    x = pose.pose.position.x;
    y = pose.pose.position.y;
    z = pose.pose.position.z;
};

void PathPlanner::setObstacleCoordinates(Eigen::VectorXd Xo,Eigen::VectorXd Yo,Eigen::VectorXd Zo, Eigen::VectorXd Sizeo) {
    assert(("Obstacle coordinates and size vectors must have the same length",Xo.size() == Yo.size() && Yo.size() == Zo.size() && Zo.size() == Sizeo.size()));
    this->Xo = Xo;
    this->Yo = Yo;
    this->Zo = Zo;
    Sizeo = Sizeo.array() + safety_margin;
    this->Sizeo = Sizeo;

    long Nobstacles = Xo.size();
    o_sensed.resize(Nobstacles);
    o_sensed_A.resize(Nobstacles);
    o_dist.resize(Nobstacles);
//    o_dist_1_2.resize(Nobstacles);
//    o_dist_1_2_inv.resize(Nobstacles);
//    o_dist_inv.resize(Nobstacles);
    x_dist.resize(Nobstacles);
    y_dist.resize(Nobstacles);
    z_dist.resize(Nobstacles);
    x_dist_2.resize(Nobstacles);
    y_dist_2.resize(Nobstacles);
    z_dist_2.resize(Nobstacles);
//    f_core.resize(Nobstacles);
//    cos_f_core.resize(Nobstacles);
//    sin_f_core.resize(Nobstacles);
//    der_core.resize(Nobstacles);
    GAUSSIAN.resize(Nobstacles);
    foxGAUSSIAN.resize(Nobstacles);
    foyGAUSSIAN.resize(Nobstacles);
    fozGAUSSIAN.resize(Nobstacles);
    A.resize(Nobstacles);
    A1.resize(Nobstacles);
    A2.resize(Nobstacles);
    sigma.resize(Nobstacles);
//    sigma_1_2.resize(Nobstacles);
//    pi_sigma.resize(Nobstacles);
//    pi_sigma_2.resize(Nobstacles);

};

void PathPlanner::setSafetyMargin(double safety_margin){
    this->safety_margin = safety_margin;
}

void PathPlanner::setHasObstacles(bool hasObstacles){
    this->hasObstacles = hasObstacles;
}

void PathPlanner::setDist_sensed_obs(double dist_sensed_obs) {
    this->dist_sensed_obs = dist_sensed_obs;
}

void PathPlanner::setSurfToBeDef(int surfToBeDef) {
    if (surfToBeDef == 1 || surfToBeDef == 2) {
        this->surfToBeDef = surfToBeDef;
    } else {
        this->surfToBeDef = 1;
        std::cout << "Invalid value for surfToBeDef. Setting it to: " << this->surfToBeDef << std::endl;
    }
}

void PathPlanner::setSurfFlag (int surfFlag){
    if (surfFlag >= 0)
        this->surfFlag = 1;
    else{
        this->surfFlag = -1;
    }
    if (this->surfFlag != surfFlag){
        std::cout << "Invalid value for surfFlag. Setting it to: " << this->surfFlag<< std::endl;
    }
};

void PathPlanner::setTangFlag (int tangFlag){
    if (tangFlag >= 0) {
        this->tangFlag = 1;
    } else {
        this->tangFlag = -1;
    }
    if (this->tangFlag != tangFlag){
        std::cout << "Invalid value for tangFlag. Setting it to: " << this->tangFlag<< std::endl;
    }
};

void PathPlanner::setDesiredYaw(double desired_yaw){
    this->desiredYaw = desired_yaw;
}

void PathPlanner::setSigmaMultiplier (double sigma_multiplier){
    this->sigma_multiplier = sigma_multiplier;
};

void PathPlanner::setXYGain(double xyGain){
    this->xyGain = xyGain;
};

void PathPlanner::setZGain(double zGain){
    this->zGain = zGain;
};

void PathPlanner::setFrameId (std::string frame_id){
    this->frame_id = frame_id;
};

void PathPlanner::setKgrad1(double Kgrad1){
    this->Kgrad1 = Kgrad1;
};

void PathPlanner::setKgrad2(double Kgrad2){
    this->Kgrad2 = Kgrad2;
};

void PathPlanner::setKtang(double Ktang){
    this->Ktang = Ktang;
};

Eigen::Vector3d PathPlanner::getVelocityVector() {
    return velocityVector;
};

double PathPlanner::getYawError() {
  return yaw_err;
};

geometry_msgs::PoseStamped PathPlanner::getRobotPose() {
    return robotPose;
};

void PathPlanner::computeObstaclesDistances (){
    double x = robotPose.pose.position.x;
    double y = robotPose.pose.position.y;
    double z = robotPose.pose.position.z;

    x_dist = -(Xo.array() - x);
    y_dist = -(Yo.array() - y);
    z_dist = -(Zo.array() - z);

    x_dist_2 = x_dist.pow(2);
    y_dist_2 = y_dist.pow(2);
    z_dist_2 = z_dist.pow(2);

    o_dist = x_dist_2 + y_dist_2 + z_dist_2;
    o_dist = o_dist.sqrt();

};

void PathPlanner::computeSensedObstacles() {
    numSensedObs = 0;
    for (int i = 0; i < o_sensed.size(); i++){
        sigma(i) = sigma_multiplier * pow(Sizeo(i),2);
        if (o_dist(i) < dist_sensed_obs) {
//            visualization_msgs::Marker obsMarker;
//            geometry_msgs::Point point;
//            point.x = Xo(i);
//            point.y = Yo(i);
//            point.z = Zo(i);
//            obsMarker.points.push_back(point);
//            obsMarker.lifetime = ros::Duration(1,0);
//            obsMarker.pose.orientation.x = 0;
//            obsMarker.pose.orientation.y = 0;
//            obsMarker.pose.orientation.z = 0;
//            obsMarker.pose.orientation.w = 1;
//            obsMarker.color.a = 1;
//            obsMarker.color.b = 1;
//            obsMarker.header.stamp = ros::Time::now();
//            obsMarker.header.frame_id = "world";
//            obsMarker.id = i;
//            obsMarker.scale.x = Sizeo(i)-safety_margin;
//            obsMarker.scale.y = Sizeo(i)-safety_margin;
//            obsMarker.scale.z = Sizeo(i)-safety_margin;
//            obsMarker.type = visualization_msgs::Marker::CUBE_LIST;
//            obsMarker.action = 0;
//
//            markerArray.markers.push_back(obsMarker);
            //o_sensed(i) = (dist_sensed_obs - o_dist(i))/dist_sensed_obs;
            o_sensed(i) = pow(o_dist(i)- dist_sensed_obs,2) / pow(dist_sensed_obs,2);

            numSensedObs++;
        } else {
            o_sensed(i) = 0;// pow(dist_sensed_obs / o_dist(1),3);
        }
    }
    //std::cout << markerArray.markers.size() << std::endl;
    //std::cout << "sizeo = " << Sizeo(1) << std::endl;
    //std::cout << "sigma = " << sigma(1) << std::endl;
    //std::cout << "numSensedObs = " << numSensedObs <<std::endl;
};

void PathPlanner::computeObstaclesAmplitude() {
    long Nobstacles = Xo.size();

    // the amplitude of the obstacle function is re-computed at each
    // iteration for each individual obstacle
    for (int o = 0; o < Nobstacles; o++) {
        if (o_sensed(o) == 0) {
            A(o) = 0;
        } else {

            // this variable is introduced to simplify computations later
            //double min_inv = 1 / (1 + cos(Sizeo(o) * M_PI / sqrt(sigma(o))));
            double min_inv = exp(pow(Sizeo(o),2)/sigma(o));
            // compute the parameters of the hyperplane tangent to f1(x,y,z) in xc yc zc
            Eigen::Vector3d gradFr;
            double fvalr;

            if (surfToBeDef == 2){
                gradFr = f2->computeGradientValue(Xo(o),Yo(o),Zo(o));
                fvalr = f2->computeFunctionValue(Xo(o),Yo(o),Zo(o));

            } else {
                gradFr = f1->computeGradientValue(Xo(o),Yo(o),Zo(o));
                fvalr = f1->computeFunctionValue(Xo(o),Yo(o),Zo(o));
            }

            Eigen::Vector3d obstacleCenter;
            obstacleCenter << Xo(o),Yo(o),Zo(o);

            if (surfFlag == 1) {

                double ar = gradFr(0);
                double br = gradFr(1);
                double cr = gradFr(2);
                double dr = -gradFr.transpose() * obstacleCenter + fvalr;
                double denom = gradFr.norm();


                // for each obstacle centered in xc, yc, zc, compute the minimum of the first surface on a sphere centered in xc, yc, zc
                // obstacles in the half-plane with y < x
                double minFr = -pow(ar,2) * Sizeo(o) / denom + ar * obstacleCenter(0) -
                               pow(br,2) * Sizeo(o) / denom + br * obstacleCenter(1) -
                               pow(cr,2) * Sizeo(o) / denom + cr * obstacleCenter(2) + dr;

                // compute a proper value for the amplitude of the Bell function
                // obstacles in the half-plane with y < x
//                A(o) = std::max(0.0, -minFr * min_inv);
                A(o) = fabs(-minFr * min_inv);
            }

            if (surfFlag == -1) {

                double al = -gradFr(0);
                double bl = -gradFr(1);
                double cl = -gradFr(2);
                double dl = gradFr.transpose() * obstacleCenter - fvalr;
                double denom = gradFr.norm();


                // for each obstacle centered in xc, yc, zc, compute the minimum of the first surface on a sphere centered in xc, yc, zc
                // obstacles in the half-plane with y < x
                double minFl = -pow(al,2) * Sizeo(o) / denom + al * obstacleCenter(0) -
                               pow(bl,2) * Sizeo(o) / denom + bl * obstacleCenter(1) -
                               pow(cl,2) * Sizeo(o) / denom + cl * obstacleCenter(2) + dl;

                // compute a proper value for the amplitude of the Bell function
                // obstacles in the half-plane with y < x
//                A(o) = std::max(0.0, -minFl * min_inv);
                A(o) = fabs(-minFl * min_inv);
            }
        }
    }
};

void PathPlanner::computeIntermediateVariables() {
    // these variables are introduced to simplify computations later
//    sigma_1_2 = sigma.sqrt();
//    o_dist_inv = o_dist.pow(-1);
//    o_dist_1_2 = o_dist.sqrt();
//    o_dist_1_2_inv = o_dist_1_2.pow(-1);
//    pi_sigma = M_PI / sigma_1_2;
//    pi_sigma_2 = pi_sigma.pow(2);
//    f_core = o_dist_1_2 * pi_sigma;
//    cos_f_core = f_core.cos();
//    sin_f_core = f_core.sin();

//    der_core = -o_sensed_A*pi_sigma*sin_f_core*o_dist_1_2_inv;

//    der_core_1 = -o_sensed_A*pi_sigma_2*cos_f_core*o_dist_inv;
//    der_core_2 << der_core;
//    der_core_3 = o_sensed_A*pi_sigma*o_dist_inv*o_dist_1_2_inv;

//    GAUSSIAN = o_sensed_A*(cos_f_core + 1);

//    foxGAUSSIAN = der_core*x_dist;
//    foyGAUSSIAN = der_core*y_dist;
//    fozGAUSSIAN = der_core*z_dist;

//    std::ostringstream filename;
//    filename << "./debug/" << x <<"_"<< y << ".txt";
//    std::string filenamestring;
//    filenamestring = filename.str();
//
//    std::ofstream myfile (filenamestring.c_str());
//    if (myfile.is_open())
//    {
//        //std::cout << "saving" << std::endl;
//        int count;
//        myfile << "Obstacle distance" << "\t" << "fozGaussian" << "\n";
//        for(count = 0; count < o_sensed.size(); count ++){
//            myfile << o_dist (count) << "\t" << fozGAUSSIAN(count) << "\n";
//        }
//        //std::cout << count << std::endl;
//        myfile.close();
//    }
//    else std::cout << "Unable to open file";

};

void PathPlanner::computeObstaclesContribution(){
    if ( o_sensed.size() > 0)
    {
        o_sensed_A = o_sensed*A.array();
    }else{
        o_sensed_A = Eigen::ArrayXd::Zero(1);
    }

    GAUSSIAN = o_sensed_A*exp(-o_dist/sigma);

    foxGAUSSIAN = -2 * o_sensed_A * x_dist/sigma * exp(-o_dist.pow(2)/sigma);
    foyGAUSSIAN = -2 * o_sensed_A * y_dist/sigma * exp(-o_dist.pow(2)/sigma);
    fozGAUSSIAN = -2 * o_sensed_A * z_dist/sigma * exp(-o_dist.pow(2)/sigma);

    fo = GAUSSIAN.sum();
    foGrad << foxGAUSSIAN.sum(),foyGAUSSIAN.sum(),fozGAUSSIAN.sum();
};

void PathPlanner::computeVelocityVector() {
    // compute the expression of the surfaces in x, y, z (function and gradient)
    f1val = f1->computeFunctionValue(x,y,z);
    Eigen::Vector3d grad1 = f1->computeGradientValue(x,y,z);

    if (grad1.norm() != 0){
        grad1.normalize();
    }

    f2val = f2->computeFunctionValue(x,y,z);
    Eigen::Vector3d grad2 = f2->computeGradientValue(x,y,z);
    if (grad2.norm() != 0){
        grad2.normalize();
    }

    //sum obstacle contribution to the surface defined by surfToBeDef
    if (surfToBeDef == 2) {
        f2val *= surfFlag;
        grad2 *= surfFlag;
        if (hasObstacles) {

            //We want to deform the trajectory along z only
            foGrad (0) = 0;
            foGrad (1) = 0;

            f2val += fo;
            grad2 += foGrad;
        }
    } else {

        f1val *= surfFlag;
        grad1 *= surfFlag;
        if (hasObstacles) {

            //We want to deform the trajectory along xy only
            foGrad(2) = 0;

            f1val += fo;
            grad1 += foGrad;
        }
    }

    // normalize the results
    double f1n = f1val;
    double f2n = f2val;
    if (grad1.norm() != 0) {
        f1n = f1val / grad1.norm();
        grad1.normalize();
    }
    if (grad2.norm() != 0) {
        f2n = f2val / grad2.norm();
        grad2.normalize();
    }
    // compute the crossproduct of the two gradients, i.e., a vector that is
    // parallel to the intersection of f1 and f2
    Eigen::Vector3d t12 = grad1.cross(grad2);
    t12 *= tangFlag;

    // compute the velocity vector by summing vectors
    velocityVector(0) = -Kgrad1*f1n*grad1(0)-Kgrad2*f2n*grad2(0)+Ktang*t12(0);
    velocityVector(1) = -Kgrad1*f1n*grad1(1)-Kgrad2*f2n*grad2(1)+Ktang*t12(1);
    velocityVector(2) = -Kgrad1*f1n*grad1(2)-Kgrad2*f2n*grad2(2)+Ktang*t12(2);
    if (velocityVector.norm() != 0) {
        velocityVector.normalize();
    }

    velocityVector(0) *= xyGain;
    velocityVector(1) *= xyGain;
    velocityVector(2) *= zGain;

}

double PathPlanner::computeYawCommand() {
    double actualYaw = tf::getYaw(robotPose.pose.orientation);

    //Update previous errors
    yaw_err_t_2 = yaw_err_t_1;
    yaw_err_t_1 = yaw_err;

    //compute current error to be in the interval (-pi,pi)
    yaw_err = desiredYaw - actualYaw;

    if (yaw_err > M_PI) {
        yaw_err -= 2 * M_PI;
    } else if (yaw_err < -M_PI){
        yaw_err += 2 * M_PI;
    }

    // three-point derivative (Lagrange Approach) f'(x) = ( f(x-2h) - 4f(x-h) + 3f(x) ) / 2h
    double step = timeStamp.toSec() - timeStamp_t_1.toSec();
    double yaw_err_d = (yaw_err_t_2 - 4 * yaw_err_t_1 + 3 * yaw_err) / 2 * step;
    if (!isnan(yaw_err)) {
        yaw_err_i += yaw_err * I_yaw;
    }
    double yaw_command =  P_yaw * yaw_err + D_yaw * yaw_err_d + yaw_err_i;
    return yaw_command;
};

void PathPlanner::run(){
    timeStamp_t_1 = timeStamp;
    timeStamp = ros::Time::now();
    if (hasObstacles && Xo.size() > 0) {
        computeObstaclesDistances();
        computeSensedObstacles();
        computeObstaclesAmplitude();
//        computeIntermediateVariables();
        computeObstaclesContribution();
    }
    computeVelocityVector();
};
