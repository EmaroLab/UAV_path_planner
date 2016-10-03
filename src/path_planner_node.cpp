#include <ros/ros.h>
#include <asctec_hl_comm/mav_ctrl.h>
#include <cmath>
#include <path_planner.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include "cylinder.h"
#include "plane.h"
#include <unistd.h>

PathPlanner* pathPlanner;
double pcRaise;
bool publish_vectorial_field;


void getParams(ros::NodeHandle &nh);

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

void octomap_cb (const octomap_msgs::Octomap map_msg);

void pose_cb (const geometry_msgs::PoseStamped pose_msg);

void publishVectorialField(ros::NodeHandle &nh);

int main (int argc, char** argv) {

//    Surface_function* f1 = new Cylinder(1.5,3,0,0);
    Surface_function* f1 = new Plane(1,0,0,0);
    Surface_function* f2 = new Plane (0,0,1,-0.4);
    pathPlanner = new PathPlanner(f1,f2);

    // Initialize ROS
    ros::init (argc, argv, "trajectory_planner_node");
    ros::NodeHandle nh("~");

    //Subscribing to topics
    ros::Subscriber octomapSub = nh.subscribe<octomap_msgs::Octomap> ("/map_in", 1, octomap_cb);
    //ros::Subscriber pointCloudSub = nh.subscribe<sensor_msgs::PointCloud2>("map_in",1,cloud_cb);
    ros::Subscriber poseSub = nh.subscribe<geometry_msgs::PoseStamped>("/pose_in",1,pose_cb);

    //Publishing topics
    ros::Publisher pub = nh.advertise<asctec_hl_comm::mav_ctrl>("/command_out",1);
    ros::Publisher markerPub = nh.advertise<visualization_msgs::Marker>("velocity_vector_marker",1);
//  ros::Publisher markerArrayPub = nh.advertise<visualization_msgs::MarkerArray>("/octomap_vis",1);
    ros::Publisher pathPublisher = nh.advertise<nav_msgs::Path>("/path",1,true);

    getParams(nh);

    if (publish_vectorial_field) {
        publishVectorialField(nh);
        return(0);
    }

    //Declaring messages
    asctec_hl_comm::mav_ctrl velocityMsg;
    asctec_hl_comm::mav_ctrl positionMsg;
    tf::Quaternion desiredRotation;
    geometry_msgs::PoseStamped robotPose;
    Eigen::Vector3d velocityVector;
    visualization_msgs::Marker visualization_vector;
    geometry_msgs::Point vectorTip;

    //Constant values of the velocity message
    velocityMsg.v_max_xy = 0.1;
    velocityMsg.v_max_z = 0.1;
    velocityMsg.type = 2;
    velocityMsg.header.frame_id = "world";

    Eigen::MatrixXd command_window (3,100);

    // Spin
    ros::Rate rate(100);

    while (ros::ok()){

        //Running the path planner
        pathPlanner->run();
        robotPose = pathPlanner->getRobotPose();
        velocityVector = pathPlanner->getVelocityVector();

        //Filling the velocity vector message
        velocityMsg.header.stamp = ros::Time::now();
        velocityMsg.z = velocityVector(2);
        velocityMsg.yaw = pathPlanner->getYawCommand();

        for (int i = 0; i < command_window.cols()-1; i++){
            command_window.col(i) << 0.95 * command_window.col(i+1);
        }

        command_window.col(command_window.cols()-1) << velocityVector;

        //If yaw error  is too high stop moving along xy plane and wait for rotation
        if (fabs(pathPlanner->getYawError()) < 0.3){
            velocityMsg.x = command_window.row(0).sum()/command_window.size();
            velocityMsg.y = command_window.row(1).sum()/command_window.size();
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

        //Publishing
        markerPub.publish(visualization_vector);
//        markerArrayPub.publish(pathPlanner->markerArray);
//        pathPlanner->markerArray.markers.clear();
        pub.publish(velocityMsg);

        ros::spinOnce ();
        rate.sleep();
    }
}

void publishVectorialField(ros::NodeHandle &nh) {

    ros::Publisher vectorialFieldPub = nh.advertise<visualization_msgs::MarkerArray>("/vectorial_field",1,true);

    Eigen::Vector3d velocityVector;
    visualization_msgs::MarkerArray vectorial_field;

    visualization_msgs::Marker visualization_vector;
    geometry_msgs::Point vectorTip;
    nav_msgs::Path path;
    path.header.frame_id = "world";
    ROS_INFO("Waiting for map message to be published");
    ros::topic::waitForMessage<octomap_msgs::Octomap>("/map_in", nh);
    ros::spinOnce();
    ROS_INFO("Map received. Plotting vectorial field");
    double step = 0.15;
    int id = 0;
    for (double x = -2; x < 2; x += step){
        for (double y = -2.5; y < 2.5; y+= step){
            pathPlanner->setRobotPose(x, y, 0.4, tf::Quaternion(0, 0, 0, 1));
            pathPlanner->run();
            velocityVector = pathPlanner->getVelocityVector();
            visualization_vector.header.frame_id = "world";
            visualization_vector.id = id;
            visualization_vector.type= visualization_msgs::Marker::ARROW;
            visualization_vector.points.clear();
            geometry_msgs::Point point;
            point.x = x;
            point.y = y;
            point.z = 0.4;
            visualization_vector.points.push_back(point);
            vectorTip.x = x + velocityVector(0);
            vectorTip.y = y + velocityVector(1);
            vectorTip.z = 0.4 + velocityVector(2);
            visualization_vector.points.push_back(vectorTip);
            visualization_vector.scale.x = 0.01;
            visualization_vector.scale.y = 0.02;
            visualization_vector.scale.z = 0.04;
            visualization_vector.color.a = 1;
            visualization_vector.color.r = 1;
            vectorial_field.markers.push_back(visualization_vector);
            //std::cout << f1->computeFunctionValue(x,y,0.4);
//            if (fabs(pathPlanner->getf1val()) < 0.1){
//                path.poses.push_back(pathPlanner->getRobotPose());
//            }
            id ++;
        }
    }
    //pathPublisher.publish(path);
    vectorialFieldPub.publish(vectorial_field);
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
    nh.param("publish_vectorial_field", publish_vectorial_field, false);
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
            if (it.getCoordinate().z() > 0.1) {
                Xo(numOccupiedLeaves) = it.getCoordinate().x();
                Yo(numOccupiedLeaves) = it.getCoordinate().y();
                Zo(numOccupiedLeaves) = it.getCoordinate().z() + pcRaise;
                Sizeo(numOccupiedLeaves) = it.getSize();
                numOccupiedLeaves++;
            }
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
