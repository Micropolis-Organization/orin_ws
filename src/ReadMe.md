- Lidar: roslaunch velodyne_pointcloud VLP-32C_points.launch 
- IMU: roslaunch wit_ros_imu wit_imu.launch port:=/dev/ttyUSB1
- Ground extractor: roslaunch pcl_ground_extractor pcl_ground_extractor.launch 

- DLO: roslaunch direct_lidar_odometry dlo.launch
- Transform publisher: roslaunch direct_lidar_odometry robot_tf_publish.launch 


- CAN: roslaunch can_pkg can_pkg.launch
- CMD_VEL to velocity and steering: 

- CAN GUI: rosrun can_pkg ros_slider_publisher.py