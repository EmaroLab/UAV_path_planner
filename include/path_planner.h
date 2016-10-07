#ifndef PATH_PLANNER_PATH_PLANNER_H
#define PATH_PLANNER_PATH_PLANNER_H

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <eigen3/Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/tf.h>
#include <cmath>
#include <string>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <visualization_msgs/MarkerArray.h>
#include "surface_function.h"


class PathPlanner {

public:

    PathPlanner();

    PathPlanner(Surface_function* f1, Surface_function* f2);

    ~PathPlanner();

    void setRobotPose(double x, double y, double z, tf::Quaternion q);

    void setRobotPose(geometry_msgs::PoseStamped pose);

    void setRobotPose(geometry_msgs::Pose pose);

    void setSurfToBeDef(int surfToBeDef);

    void setSurfFlag(int surfFlag);

    void setTangFlag(int tangFlag);

    void setSigmaMultiplier(double sigma_multiplier);

    void setFrameId(std::string frame_id);

    void setObstacleCoordinates(Eigen::VectorXd Xo, Eigen::VectorXd Yo, Eigen::VectorXd Zo, Eigen::VectorXd Sizeo);

    void setSafetyMargin (double safety_margin);

    void setHasObstacles(bool hasObstacles);

    void setDist_sensed_obs(double dist_sensed_obs);

    void setXYGain(double xyGain);

    void setZGain(double zGain);

    void setKgrad1(double Kgrad1);

    void setKgrad2(double Kgrad2);

    void setKtang(double Ktang);

    void setDesiredYaw(double desired_yaw);

    void run();

    Eigen::Vector3d getVelocityVector();

    geometry_msgs::PoseStamped getRobotPose();

    double computeYawCommand();

    double getYawError();

private:

    void setDefaultParameters();

    void computeObstaclesDistances();

    void computeSensedObstacles();

    void computeObstaclesAmplitude();

    void computeIntermediateVariables();

    void computeObstaclesContribution();

    void computeVelocityVector();

    Surface_function* f1;
    Surface_function* f2;

    double f1val;
    double f2val;

    //Coordinate and size of obstacles
    Eigen::VectorXd Xo;
    Eigen::VectorXd Yo;
    Eigen::VectorXd Zo;
    Eigen::VectorXd Sizeo;
    //Obstacles are inflated by this factor according to their size
    double safety_margin;

    //Current robot pose
    geometry_msgs::PoseStamped robotPose;
    std::string frame_id;

    //Variables used to store robot position and make the code more neat
    double x;
    double y;
    double z;

    //Amplitude of bell functions
    Eigen::ArrayXd sigma;

    //Vectors to store amplitude of obstacles wrt current robot pose
    Eigen::VectorXd A1;
    Eigen::VectorXd A2;
    Eigen::VectorXd A;


    //Arrays to store the distance and the squared distance of each obstacle from the robot
    Eigen::ArrayXd x_dist;
    Eigen::ArrayXd y_dist;
    Eigen::ArrayXd z_dist;
    Eigen::ArrayXd x_dist_2;
    Eigen::ArrayXd y_dist_2;
    Eigen::ArrayXd z_dist_2;
    Eigen::ArrayXd o_dist;

    //obstacles farther than this threshold are not considered
    double dist_sensed_obs;

    //Logical Array to determine which obstacles to consider
    Eigen::ArrayXd o_sensed;
    int numSensedObs;

    //Intermediate variables introduce to simplify computation
//    Eigen::ArrayXd o_dist_inv;
//    Eigen::ArrayXd o_dist_1_2;
//    Eigen::ArrayXd o_dist_1_2_inv;
//    Eigen::ArrayXd f_core;
//    Eigen::ArrayXd cos_f_core;
//    Eigen::ArrayXd sin_f_core;
    Eigen::ArrayXd o_sensed_A;
//    Eigen::ArrayXd der_core;
    Eigen::ArrayXd GAUSSIAN;
    Eigen::ArrayXd foxGAUSSIAN;
    Eigen::ArrayXd foyGAUSSIAN;
    Eigen::ArrayXd fozGAUSSIAN;
//    Eigen::ArrayXd sigma_1_2;
//    Eigen::ArrayXd pi_sigma;
//    Eigen::ArrayXd pi_sigma_2;

    //Obstacle contributions and gradient
    double fo;
    Eigen::Vector3d foGrad;

    //Output velocity vector and yaw command
    Eigen::Vector3d velocityVector;
    double desiredYaw;

    //Yaw errors at instants t, t-1 and t-2 and integral for PID
    double yaw_err;
    double yaw_err_t_1;
    double yaw_err_t_2;
    double yaw_err_i;

    //Timestamps
    ros::Time timeStamp;
    ros::Time timeStamp_t_1;

    //Velocity gains along xy and z
    double xyGain;
    double zGain;

    //Gains for tangent and gradient contributions
    double Kgrad1;
    double Kgrad2;
    double Ktang;

    //PID gains for rotational velocity
    double P_yaw;
    double I_yaw;
    double D_yaw;

    bool hasObstacles;
    int surfFlag;     //determines the surface f1 or f2=-f1 to be used to define the path
    int tangFlag; // determines the direction along which the robot follows the path
    double sigma_multiplier; //The higher this coefficient the higher the influence of the obstacles on the path
    int surfToBeDef; //Defines the surface to be deformed. It can be either 1 or 2.

};

#endif //PATH_PLANNER_PATH_PLANNER_H
