#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <cmath>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

geometry_msgs::PoseStamped robotPose;
//Obstacle centers and size
Eigen::VectorXd Xo(1);
Eigen::VectorXd Yo(1);
Eigen::VectorXd Zo(1);
Eigen::VectorXd Sizeo(1);
bool hasObstacles = false;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    long Nobstacles = cloud.points.size();
    if (Nobstacles == 0){
        hasObstacles = false;
        return;
    }
    hasObstacles = true;
    Xo.resize(Nobstacles);
    Yo.resize(Nobstacles);
    Zo.resize(Nobstacles);
    Sizeo.resize(Nobstacles);
    int i = 0;
    pcl::PointCloud<pcl::PointXYZ>::iterator it;
    for (it  = cloud.begin(); it != cloud.end(); it++){
//        if (fabs(it.base()->z-0.7)< 0.5) {
            Xo(i) = it.base()->x;
            Yo(i) = it.base()->y;
            Zo(i) = it.base()->z;
            Sizeo(i) = 0.25;
            i++;
//        }
    }
//    std::cout << "Xo: " << Xo << std::endl;
//    std::cout << "Yo: " << Yo << std::endl;
//    std::cout << "Zo: " << Zo << std::endl << std::endl;
//
//    Xo.conservativeResize(i);
//    Yo.conservativeResize(i);
//    Zo.conservativeResize(i);
//    Sizeo.resize(i);
//    std::cout << "Xo: " << Xo << std::endl;
//    std::cout << "Yo: " << Yo << std::endl;
//    std::cout << "Zo: " << Zo << std::endl << std::endl;

}

void octomap_cb (const octomap_msgs::Octomap map_msg) {
    octomap::AbstractOcTree* abstractOcTree = octomap_msgs::fullMsgToMap(map_msg);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(abstractOcTree);
    long Nobstacles = octree->getNumLeafNodes();
    if (Nobstacles == 0){
        hasObstacles = false;
        return;
    }
    hasObstacles = true;
    Xo.resize(Nobstacles);
    Yo.resize(Nobstacles);
    Zo.resize(Nobstacles);
    Sizeo.resize(Nobstacles);

    int i = 0;
    for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
                end=octree->end_leafs(); it!= end; ++it)
    {
        if (octree->isNodeOccupied(it.operator*())) {
            Xo(i) = it.getCoordinate().x();
            Yo(i) = it.getCoordinate().y();
            Zo(i) = it.getCoordinate().z();
            Sizeo(i) = it.getSize();
            i++;
        }
    }

    std::cout << "Num Leaves = " << Nobstacles << std::endl;
    std::cout << "Num occupied Leaves = " << i << std::endl;
    Xo.conservativeResize(i);
    Yo.conservativeResize(i);
    Zo.conservativeResize(i);

}

void pose_cb (const geometry_msgs::PoseStamped pose_msg)
{
    robotPose = pose_msg;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "trajectory_planner_node");
    ros::NodeHandle nh;

//    ros::Subscriber octomapSub = nh.subscribe<octomap_msgs::Octomap> ("/octomap_full", 1, octomap_cb);
    ros::Subscriber pointCloudSub = nh.subscribe<sensor_msgs::PointCloud2>("/octomap_point_cloud_centers",1,cloud_cb);
    ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseStamped>("/firefly/ground_truth/pose",1,pose_cb);

    ros::Publisher pub = nh.advertise<geometry_msgs::PoseStamped> ("/firefly/command/pose", 1);

    Eigen::Matrix3f Q;
    Eigen::Vector3f P;
    Eigen::Matrix3f H;
    double S;

    Eigen::VectorXd A1(1);
    Eigen::VectorXd A2(1);
    Eigen::VectorXd A(1);
    bool run_time = true;
// initialize the surfFlag, which determines the surface f1 or f2=-f1
// to be used to define the path
    int surfFlag = 1;

// initialize the tangFlag, which determines the direction along which the
// robot follows the path
    int tangFlag = -1;

// define the surface to be modified in presence of obstacles, either f1=0
// or f2=0
    int surfToBeDef = 1;

    Eigen::Quaterniond q (1,0,0,0);

    //Parameters of the circular trajectory to be followed
    double a = 2;
    double b = 2;
    double x0 = 0;
    double y0 = 0;
    double fo;
    double fox;
    double foy;
    double foz;
    double foxx;
    double foyy;
    double fozz;
    double foxy;

    Q << 1/pow(a,2), 0, 0, 0, 1/pow(b,2), 0, 0, 0, 0;
    Q *= 20;

    P << -2*x0/pow(a,2), -2*y0/pow(b,2), 0;
    P *= 20;

    S = 20*(pow(x0,2)/pow(a,2) + pow(y0,2)/pow(b,2) -1);

    H << 2 * Q(0, 0), Q(0, 1), Q(0, 2), Q(1, 0), 2 * Q(1, 1), Q(1, 2), Q(2, 0), Q(2, 1), 2 * Q(2, 2);
//    H *= 20;


    // Spin
    ros::Rate rate(100);

    while (ros::ok()){
        //Set robot Pose
        double x = robotPose.pose.position.x;
        double y = robotPose.pose.position.y;
        double z = robotPose.pose.position.z;
        int numSensedObs = 0;
        if (hasObstacles && Xo.size() > 0) {

            // these variables are introduced to simplify computations later
//        Eigen::VectorXd x_dist = x * Eigen::VectorXd::Ones(Xo.size())  - Xo;
//        Eigen::VectorXd y_dist = y * Eigen::VectorXd::Ones(Xo.size())  - Yo;
//        Eigen::VectorXd z_dist = z * Eigen::VectorXd::Ones(Xo.size())  - Zo;

            //ComputeObstacleDistances
            Eigen::ArrayXd x_dist;
            x_dist = -(Xo.array() - x);
            Eigen::ArrayXd y_dist;
            y_dist = -(Yo.array() - y);
            Eigen::ArrayXd z_dist;
            z_dist = -(Zo.array() - z);

            Eigen::ArrayXd x_dist_2;
            x_dist_2 = x_dist.pow(2);
            Eigen::ArrayXd y_dist_2;
            y_dist_2 = y_dist.pow(2);
            Eigen::ArrayXd z_dist_2;
            z_dist_2 = z_dist.pow(2);

            Eigen::ArrayXd o_dist = (x_dist_2 + y_dist_2 + z_dist_2);
            Eigen::ArrayXd::Index minIndex;

            // these variables record, at each iteration, the distance of the closer
            // obstacle
            double min_dist = o_dist.minCoeff(&minIndex);
            double x_near = Xo(minIndex) - x;
            double y_near = Yo(minIndex) - y;
            double z_near = Zo(minIndex) - z;

            // sigma define the influence distance of obstacles: the obstacle is
            // perceived if and only if o_dist < sigma
            double sigma;

            Eigen::ArrayXi o_sensed(o_dist.size());
            for (int i = 0; i < o_sensed.size(); i++){
                sigma = 15 * pow(Sizeo(i),2);
                if (o_dist(i) <= sigma) {
                    o_sensed(i) = 1;
                    numSensedObs++;
                } else {
                    o_sensed(i) = 0;
                }
            }//TODO Check sigma value at the end of this loop
            std::cout << "Obstacle Sensed = " << numSensedObs << std::endl <<std::endl;
            if (run_time) {

                long Nobstacles = Xo.size();

                // the amplitude of the obstacle function is re-computed at each
                // iteration for each individual obstacle
                A.resize(Nobstacles);
                for (int o = 0; o < Nobstacles; o++) {
                    if (o_sensed(o) == 0) {
                        A(o) = 0;
                    } else {
                        Eigen::Vector3f obstacleCenter;
                        obstacleCenter << Xo(o),Yo(o),Zo(o);

                        // this variable is introduced to simplify computations later
                        double min_inv = 1 / (1 + cos(Sizeo(o) * M_PI / sqrt(sigma)));

                        // compute the parameters of the hyperplane tangent to f1(x,y,z) in xc yc zc
                        Eigen::Vector3f gradFr;
                        gradFr = H * obstacleCenter + P;

                        //fvalr = [xc yc zc] * Q * [xc yc zc]' + P' * [xc yc zc]' + S
                        double fvalr;
                        fvalr = obstacleCenter.transpose() * Q * obstacleCenter;
                        fvalr += (P.transpose() * obstacleCenter + S);

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
                            A(o) = std::max(0.0, -minFr * min_inv);

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
                            A(o) = std::max(0.0, -minFl * min_inv);
                        }
                    }
                }
            }else{
                //if the run_time parameter is false, all parameters are used: A1 is used when surfFlag==1, A2 is used when surfFlag==2.
                if (surfFlag == 1) {
                    A = A1;
                } else {
                    A = A2;
                }
            }


            // these variables are introduced to simplify computations later
            double sigma_1_2 = sqrt(sigma);
            Eigen::ArrayXd o_dist_inv(o_dist.size());
            o_dist_inv = o_dist.pow(-1);
            Eigen::ArrayXd o_dist_1_2(o_dist.size());
            o_dist_1_2 = o_dist.sqrt();
            Eigen::ArrayXd o_dist_1_2_inv(o_dist.size());
            o_dist_1_2_inv = o_dist_1_2.pow(-1);
            double pi_sigma = M_PI / sigma_1_2;
            double pi_sigma_2 = pow(pi_sigma,2);
            Eigen::ArrayXd f_core = o_dist_1_2 * pi_sigma;
            Eigen::ArrayXd cos_f_core (f_core.size());
            cos_f_core = f_core.cos();
            Eigen::ArrayXd sin_f_core (f_core.size());
            sin_f_core = f_core.sin();

            Eigen::ArrayXd o_sensed_A;
            if ( o_sensed.size() > 0)
            {
                o_sensed_A = o_sensed.cast<double>()*A.array();
            }else{
                o_sensed_A = Eigen::ArrayXd::Zero(1);
            }

            Eigen::ArrayXd der_core (o_sensed_A.size());
            der_core = -o_sensed_A*pi_sigma*sin_f_core*o_dist_1_2_inv;
            Eigen::ArrayXd der_core_1 (o_sensed_A.size());
            der_core_1 = -o_sensed_A*pi_sigma_2*cos_f_core*o_dist_inv;
            Eigen::ArrayXd der_core_2 (o_sensed_A.size());
            der_core_2 << der_core;
            Eigen::ArrayXd der_core_3 (o_sensed_A.size());
            der_core_3 = o_sensed_A*pi_sigma*o_dist_inv*o_dist_1_2_inv;

            Eigen::ArrayXd GAUSSIAN = o_sensed_A*(cos_f_core + 1);

            Eigen::ArrayXd foxGAUSSIAN = der_core*x_dist;
            Eigen::ArrayXd foyGAUSSIAN = der_core*y_dist;
            Eigen::ArrayXd fozGAUSSIAN = der_core*z_dist;

            //     foxxGAUSSIAN= der_core_1 .* x_dist_2 + der_core_2 + der_core_3.*x_dist_2;
            //     foxyGAUSSIAN = der_core_1 .* x_dist.*y_dist + der_core_3.*x_dist.*y_dist;
            //     foyyGAUSSIAN= der_core_1 .* y_dist_2 + der_core_2 + der_core_3.*y_dist_2;

            // sum individual contributions
            fo = GAUSSIAN.sum();
            fox = foxGAUSSIAN.sum();
            foy = foyGAUSSIAN.sum();
            foz = fozGAUSSIAN.sum();

            foxx = 0;//sum(foxxGAUSSIAN);
            foxy = 0;//sum(foxyGAUSSIAN);
            foyy = 0;//sum(foyyGAUSSIAN);
        } else{
            fo = 0;
            fox = 0;
            foy = 0;
            foz = 0;

            foxx = 0;
            foxy = 0;
            foyy = 0;
        }

        // compute the expression of the surface f1 in x, y, z (function and gradient)
        double f1 = pow(x/a,2) + pow(y/b,2) -1;
        double f1x = 2/pow(a,2)*x;
        double f1y = 2/pow(b,2)*y;
        double f1z = 0;

//        double f1 = x;
//        double f1x = 1;
//        double f1y = 0;
//        double f1z = 0;

        double f1xx = 0;
        double f1xy = 0;
        double f1yy = 0;

        f1 = surfFlag*f1;
        f1x = surfFlag*f1x;
        f1y = surfFlag*f1y;
        f1z = surfFlag*f1z;
        f1xx = surfFlag*f1xx;
        f1xy = surfFlag*f1xy;
        f1yy = surfFlag*f1yy;

        // compute the expression of the surface f2 in x, y, z (function and gradient)
        double altitude = 0.7;
        double f2 = z-altitude;
        double f2x=0;
        double f2y=0;
        double f2z=1;

        if (numSensedObs != 0) {
            // sum up f1 and obstacle contributions
            f1 = f1 + fo;
            f1x = f1x + fox;
            f1y = f1y + foy;
            f1z = f1z + foz;

            f1xx = f1xx + foxx;
            f1xy = f1xy + foxy;
            f1yy = f1yy + foyy;
        }
        // normalize the results
        Eigen::Vector3d grad1;
        Eigen::Vector3d grad2;
        grad1 << f1x,f1y,f1z;
        grad2 << f2x,f2y,f2z;
        double f1n = f1;
        double f2n = f2;
        if (grad1.norm() != 0) {
            f1n = f1 / grad1.norm();
            grad1.normalize();
        }
        if (grad2.norm() != 0) {
            f2n = f2 / grad2.norm();
            grad2.normalize();
        }
        // compute the crossproduct of the two gradients, i.e., a vector that is
        // parallel to the intersection of f1 and f2
        Eigen::Vector3d t12 = tangFlag * grad1.cross(grad2);
        Eigen::Vector3d velocityVector;

        // compute the velocity vector by summing vectors
        velocityVector(0) = -0.1*f1n*grad1(0)-0.1*f2n*grad2(0)+0.1*t12(0);
        velocityVector(1) = -0.1*f1n*grad1(1)-0.1*f2n*grad2(1)+0.1*t12(1);
        velocityVector(2) = -0.1*f1n*grad1(2)-0.1*f2n*grad2(2)+0.1*t12(2);
        if (velocityVector.norm() != 0) {
            velocityVector.normalize();
        }
        double vGain = 0.2;
        velocityVector *= vGain;
        double yaw = atan2(velocityVector(1),velocityVector(0));
        tf::Quaternion desiredRotation;
        desiredRotation.setRPY(0,0,yaw);
        geometry_msgs::PoseStamped velocityMsg;
        velocityMsg.header.frame_id = "world";
        velocityMsg.header.stamp = ros::Time::now();
        velocityMsg.pose.position.x = x + velocityVector(0);
        velocityMsg.pose.position.y = y + velocityVector(1);
        velocityMsg.pose.position.z = altitude;
        velocityMsg.pose.orientation.x = desiredRotation.x();
        velocityMsg.pose.orientation.y = desiredRotation.y();
        velocityMsg.pose.orientation.z = desiredRotation.z();
        velocityMsg.pose.orientation.w = desiredRotation.w();
        pub.publish(velocityMsg);
        ros::spinOnce ();
        rate.sleep();
    }
}
