# UAV_path_planner
ROS package to generate a path to navigate a UAV in presence of obstacles.

**DEPENDENCIES** <br />
[octomap_server](http://wiki.ros.org/octomap_server)<br />
[freenect_stack](http://wiki.ros.org/freenect_stack)<br />
[hector_trajectory_server](http://wiki.ros.org/hector_trajectory_server) (only used in recording mode)<br />

**SUBSCRIBED TOPICS**<br />
_octomap_msgs/Octomap /map_in_ : input Octomap<br />
_geometry_msgs/PoseStamped /pose_in _: pose of the robot<br />

**PUBLISHED TOPICS**<br />
_asctec_hl_comm/mav_ctr /command_out_ : output velocity command to flight the drone<br />
_visualization_msgs/Marker velocity_vector_marker_ : marker to visualized the sent command in Rviz<br />
_visualization_msgs/MarkerArray /vector_field_ : marker Array to visualize the vector field in Rviz (only published if the publish_vector_field parameter is set to true. After publishing the node is killed)<br />

**USAGE**<br />
Remap the topic names in the launch files accordingly with your topics.Adjust the parameters as needed.<br />
Launch the path_planner.launch.<br />

**LAUNCH PARAMETERS**<br />
_publish_vector_field_ : if set to true publishes a vector field to be visualized on Rviz. It requires an Octomap to be published. After publishing th enode is killed. Just set to true for testing purposes.<br />
_record_trajectory_ : set to true if you want to save a rosbag with your experiment. Requires the hector_trajectory_server to be installed. The bags are saved in the folder bags/trajectories inside the package.<br />