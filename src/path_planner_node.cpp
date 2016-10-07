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
bool publish_vector_field;


void getParams(ros::NodeHandle &nh);

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

void octomap_cb (const octomap_msgs::Octomap map_msg);

void pose_cb (const geometry_msgs::PoseStamped pose_msg);

void publishVectorField(ros::NodeHandle &nh);

int main (int argc, char** argv) {

    //TODO find a way to pass the surfaces via argument or launch
    //Initializing the path planner with the two surfaces which define the path
    Surface_function* f1 = new Cylinder(1,1,0,0.3,20);
//    Surface_function* f1 = new Plane(1,0,0,0,20);
    Surface_function* f2 = new Plane (0,0,1,-0.4,20);
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

    getParams(nh);

    if (publish_vector_field) {
        publishVectorField(nh);
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
    velocityMsg.type = asctec_hl_comm::mav_ctrl::velocity;
    velocityMsg.header.frame_id = "world";

    std::vector<asctec_hl_comm::mav_ctrl> command_window(1);

    // Spin
    ros::Rate rate(100);

    while (ros::ok()){


        //Running the path planner
        pathPlanner->run();
        robotPose = pathPlanner->getRobotPose();
        velocityVector = pathPlanner->getVelocityVector();

        //Filling the velocity vector message
        velocityMsg.header.stamp = ros::Time::now();

        //LowPass filter via sliding window. The final command is an average of the commands
        //sent over the last few seconds (tunable)
        double xSum = 0;
        double ySum = 0;
        double zSum = 0;

        //Check obsolete messages. At the end of this loop i is the index of the oldest considered message
        int i = command_window.size() -1;
        while (ros::Time::now() - command_window[i].header.stamp > ros::Duration(3.0) && i > 0){
            i--;
        }

        //Truncate the vector in order to keep non-obsolete message plus one slot for the incoming message
        command_window.resize(i+2);

        //Perfom the sliding and sum up for average
        for (i++; i > 0; i--){
            command_window[i] = command_window[i-1];
            xSum += command_window[i].x;
            ySum += command_window[i].y;
            zSum += command_window[i].z;
        }

        //Add incoming message to the window and add its contibution
        command_window[0].x = velocityVector(0);
        command_window[0].y = velocityVector(1);
        command_window[0].z = velocityVector(2);
        command_window[0].header.stamp = ros::Time::now();

        xSum += velocityVector(0);
        ySum += velocityVector(1);
        zSum += velocityVector(2);

        //Output message as average of the messages in the window
        velocityMsg.x = xSum / command_window.size();
        velocityMsg.y = ySum / command_window.size();
        velocityMsg.z = zSum / command_window.size();

        velocityMsg.yaw = pathPlanner->getYawCommand();

        //Topic to visualize the velocity vector in Rviz
        visualization_vector.header.frame_id = "world";
        visualization_vector.points.clear();
        visualization_vector.points.push_back(robotPose.pose.position);
        vectorTip.x = robotPose.pose.position.x + velocityMsg.x*7;
        vectorTip.y = robotPose.pose.position.y + velocityMsg.y*7;
        vectorTip.z = robotPose.pose.position.z + velocityMsg.z*7;
        visualization_vector.points.push_back(vectorTip);
        visualization_vector.scale.x = 0.03;
        visualization_vector.scale.y = 0.06;
        visualization_vector.scale.z = 0.1;
        visualization_vector.color.a = 1;
        visualization_vector.color.r = 1;

        //If yaw error  is too high stop moving along xy plane and wait for rotation
        if (fabs(pathPlanner->getYawError()) > 1){
            velocityMsg.x = 0;
            velocityMsg.y = 0;
        }
        
        //Publishing
        markerPub.publish(visualization_vector);
//        markerArrayPub.publish(pathPlanner->markerArray);
//        pathPlanner->markerArray.markers.clear();
        pub.publish(velocityMsg);
        ros::spinOnce ();
        rate.sleep();
    }
}

void publishVectorField(ros::NodeHandle &nh) {

    ros::Publisher vectorFieldPub = nh.advertise<visualization_msgs::MarkerArray>("/vector_field",1,true);

    Eigen::Vector3d velocityVector;
    visualization_msgs::MarkerArray vector_field;
    visualization_msgs::Marker visualization_vector;
    geometry_msgs::Point vectorTip;

    ROS_INFO("Waiting for map message to be published");
    ros::topic::waitForMessage<octomap_msgs::Octomap>("/map_in", nh);
    ros::spinOnce();

    ROS_INFO("Map received. Plotting vector field");
    double step = 0.15;
    int id = 0;

    //TODO vector field is now visualized at a fixed height. Make it more flexible
    for (double x = -2; x < 2; x += step){
        for (double y = -2.5; y < 2.5; y+= step){
            pathPlanner->setRobotPose(x, y, 0.4, tf::Quaternion(0, 0, 0, 1));
            pathPlanner->run();
            velocityVector = pathPlanner->getVelocityVector();
            visualization_vector.header.frame_id = "world";
            visualization_vector.id = id;
            visualization_vector.type = visualization_msgs::Marker::ARROW;
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
            vector_field.markers.push_back(visualization_vector);
            id++;
        }
    }

    while (vectorFieldPub.getNumSubscribers() == 0){
        ROS_INFO("Waiting for subscribers to receive the vector field message");
        sleep(1);
    }

    //I have to publish several times for a remote machine to receive the message.
    //TODO Find a better solution
    for (int i = 0; i < 1000; i ++){
        vectorFieldPub.publish(vector_field);
    }
    ROS_INFO("Vector field published");
}

void getParams(ros::NodeHandle &nh) {
    //Path Planner Parameters
    int surfToBeDef;
    int surfFlag;
    int tangFlag;
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
    nh.param("publish_vector_field", publish_vector_field, false);
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
            if (it.getCoordinate().z() > 0.25) {
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

    Xo.conservativeResize(numOccupiedLeaves);
    Yo.conservativeResize(numOccupiedLeaves);
    Zo.conservativeResize(numOccupiedLeaves);
    Sizeo.conservativeResize(numOccupiedLeaves);
    pathPlanner->setObstacleCoordinates(Xo,Yo,Zo,Sizeo);
}

void pose_cb (const geometry_msgs::PoseStamped pose_msg) {
    pathPlanner->setRobotPose(pose_msg);
//    if (pose_msg.pose.position.y < -1.5){
//        pathPlanner->setTangFlag(1);
//        //pathPlanner->setSurfFlag(1);
//    }
//    if (pose_msg.pose.position.y > 1.2){
//        pathPlanner->setTangFlag(-1);
//        //pathPlanner->setSurfFlag(-1);
//    }

}
