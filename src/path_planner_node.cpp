#include <ros/ros.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <cmath>
#include <path_planner.h>
#include <visualization_msgs/Marker.h>
#include "cylinder.h"
#include "plane.h"

PathPlanner* pathPlanner;
double pcRaise;

void getParams(ros::NodeHandle &nh);

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

void octomap_cb (const octomap_msgs::Octomap map_msg);

void pose_cb (const geometry_msgs::PoseStamped pose_msg);

int main (int argc, char** argv) {

//    Surface_function* f1 = new Cylinder(1,1,0,0);
    Surface_function* f1 = new Plane(1,0,0,0);
    Surface_function* f2 = new Plane (0,0,1,-0.7);
    pathPlanner = new PathPlanner(f1,f2);

    // Initialize ROS
    ros::init (argc, argv, "trajectory_planner_node");
    ros::NodeHandle nh("~");
    ros::Subscriber octomapSub = nh.subscribe<octomap_msgs::Octomap> ("/map_in", 1, octomap_cb);
    //ros::Subscriber pointCloudSub = nh.subscribe<sensor_msgs::PointCloud2>("map_in",1,cloud_cb);
    ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseStamped>("/pose_in",1,pose_cb);

    ros::Publisher pub = nh.advertise<asctec_hl_comm::mav_ctrl>("/command_out",1);
    ros::Publisher markerPub = nh.advertise<visualization_msgs::Marker>("velocity_vector_marker",1);
    getParams(nh);

    asctec_hl_comm::mav_ctrl velocityMsg;
    asctec_hl_comm::mav_ctrl positionMsg;

    geometry_msgs::PoseStamped robotPose;
    Eigen::Vector3d velocityVector;
    tf::Quaternion desiredRotation;

    visualization_msgs::Marker visualization_vector;
    geometry_msgs::Point vectorTip;

    // Spin
    ros::Rate rate(100);

    while (ros::ok()){
        pathPlanner->run();
        robotPose = pathPlanner->getRobotPose();
        velocityVector = pathPlanner->getVelocityVector();
        velocityMsg.header.frame_id = "world";
        velocityMsg.header.stamp = ros::Time::now();
        velocityMsg.type = 2;

        velocityMsg.z = velocityVector(2);
        velocityMsg.yaw = pathPlanner->getYawCommand();
        velocityMsg.v_max_xy = 0.1;
        velocityMsg.v_max_z = 0.1;

        if (fabs(pathPlanner->getYawError()) < 0.3){
            velocityMsg.x = velocityVector(0);
            velocityMsg.y = velocityVector(1);
        }else{
            velocityMsg.x = 0;
            velocityMsg.y = 0;
        }

        //Topic to visualize the velocity vector in Rviz
        visualization_vector.header.frame_id = "world";
        visualization_vector.points.clear();
        visualization_vector.points.push_back(robotPose.pose.position);
        vectorTip.x = robotPose.pose.position.x + velocityVector(0)*5;
        vectorTip.y = robotPose.pose.position.y + velocityVector(1)*5;
        vectorTip.z = robotPose.pose.position.z + velocityVector(2)*5;
        visualization_vector.points.push_back(vectorTip);
        visualization_vector.scale.x = 0.03;
        visualization_vector.scale.y = 0.06;
        visualization_vector.scale.z = 0.1;
        visualization_vector.color.a = 1;
        visualization_vector.color.r = 1;
        markerPub.publish(visualization_vector);

        pub.publish(velocityMsg);
        ros::spinOnce ();
        rate.sleep();
    }
}

void getParams(ros::NodeHandle &nh) {
    //Trajectory Planner Parameters
    int surfToBeDef;
    int surfFlag;
    int tangFlag;
    bool run_time;
    std::string frame_id;
    double sigma_multiplier;
    double xyGain;
    double zGain;
    double Kgrad1;
    double Kgrad2;
    double Ktang;
    double dist_sensed_obs;
    double safety_margin;

    nh.param("surfFlag", surfFlag,1);
    nh.param("surfToBeDef", surfToBeDef,1);
    nh.param("tangFlag",tangFlag,1);
    nh.param("run_time",run_time,true);
    nh.param<std::string>("frame_id",frame_id, "world");
    nh.param("sigma_multiplier",sigma_multiplier,11.0);
    nh.param("xyGain",xyGain,0.1);
    nh.param("zGain",zGain,0.1);
    nh.param("Kgrad1",Kgrad1,0.1);
    nh.param("Kgrad2",Kgrad2,0.1);
    nh.param("Ktang",Ktang,0.1);
    nh.param("dist_sensed_obs",dist_sensed_obs,1.0);
    nh.param("safety_margin",safety_margin,0.2);

    pathPlanner->setFrameId(frame_id);
    pathPlanner->setSurfFlag(surfFlag);
    pathPlanner->setSurfToBeDef(surfToBeDef);
    pathPlanner->setTangFlag(tangFlag);
    pathPlanner->setRunTime(run_time);
    pathPlanner->setSigmaMultiplier(sigma_multiplier);
    pathPlanner->setXYGain(xyGain);
    pathPlanner->setZGain(zGain);
    pathPlanner->setKgrad1(Kgrad1);
    pathPlanner->setKgrad2(Kgrad2);
    pathPlanner->setKtang(Ktang);
    pathPlanner->setDist_sensed_obs(dist_sensed_obs);
    pathPlanner->setSafetyMargin(safety_margin);

    //Parameter to raise the point cloud along z. Used to avoid crashes at testing time.
    // Set to 0 for real implementation
    nh.param("pcRaise",pcRaise,0.0);

    std::cout<< "surfFlag = " << surfFlag << std::endl;
    std::cout<< "surfToBeDef = " << surfToBeDef << std::endl;
    std::cout<< "tangFlag = " << tangFlag << std::endl;
    std::cout<< "frame_id = " << frame_id << std::endl;
    std::cout<< "run_time = " << run_time << std::endl;
    std::cout<< "sigma_multiplier = " << sigma_multiplier << std::endl;
    std::cout<< "xyGain = " << xyGain << std::endl;
    std::cout<< "zGain = " << zGain << std::endl;
    std::cout<< "Kgrad1 = " << Kgrad1 << std::endl;
    std::cout<< "Kgrad2 = " << Kgrad1 << std::endl;
    std::cout<< "Ktang = " << Ktang << std::endl;
    std::cout<< "pcRaise = " << pcRaise << std::endl;
    std::cout << "safety_margin = " << safety_margin << std::endl;
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg){

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    long Nobstacles = cloud.points.size();
    if (Nobstacles == 0){
        pathPlanner->setHasObstacles(false);
        return;
    }
    pathPlanner->setHasObstacles(true);
    Eigen::ArrayXd Xo(Nobstacles);
    Eigen::ArrayXd Yo(Nobstacles);
    Eigen::ArrayXd Zo(Nobstacles);
    Eigen::ArrayXd Sizeo(Nobstacles);

    int i = 0;
    pcl::PointCloud<pcl::PointXYZ>::iterator it;
    for (it  = cloud.begin(); it != cloud.end(); it++){
        Xo(i) = it.base()->x;
        Yo(i) = it.base()->y;
        Zo(i) = it.base()->z + pcRaise;
        Sizeo(i) = 0.25;
        i++;
    }
    pathPlanner->setObstacleCoordinates(Xo,Yo,Zo,Sizeo);
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
        pathPlanner->setHasObstacles(false);
        return;
    }
    pathPlanner->setHasObstacles(true);
    Eigen::ArrayXd Xo(Nobstacles);
    Eigen::ArrayXd Yo(Nobstacles);
    Eigen::ArrayXd Zo(Nobstacles);
    Eigen::ArrayXd Sizeo(Nobstacles);

    int numOccupiedLeaves = 0;
    for(octomap::OcTree::leaf_iterator it = octree->begin_leafs(),
                end=octree->end_leafs(); it!= end; ++it)
    {
        if (octree->isNodeOccupied(it.operator*())) {
            Xo(numOccupiedLeaves) = it.getCoordinate().x();
            Yo(numOccupiedLeaves) = it.getCoordinate().y();
            Zo(numOccupiedLeaves) = it.getCoordinate().z()+ pcRaise;
            Sizeo(numOccupiedLeaves) = it.getSize();
            numOccupiedLeaves++;
        }
    }

    if (numOccupiedLeaves == 0){
        pathPlanner->setHasObstacles(false);
        return;
    }

//    std::cout << "Num Leaves = " << Nobstacles << std::endl;
//    std::cout << "Num occupied Leaves = " << numOccupiedLeaves << std::endl;
    Xo.conservativeResize(numOccupiedLeaves);
    Yo.conservativeResize(numOccupiedLeaves);
    Zo.conservativeResize(numOccupiedLeaves);
    Sizeo.conservativeResize(numOccupiedLeaves);
    pathPlanner->setObstacleCoordinates(Xo,Yo,Zo,Sizeo);
}

void pose_cb (const geometry_msgs::PoseStamped pose_msg) {
    pathPlanner->setRobotPose(pose_msg);
    if (pose_msg.pose.position.y < -1.5){
        pathPlanner->setTangFlag(1);
        //pathPlanner->setSurfFlag(1);
    }
    if (pose_msg.pose.position.y > 1.2){
        pathPlanner->setTangFlag(-1);
        //pathPlanner->setSurfFlag(-1);
    }

}
