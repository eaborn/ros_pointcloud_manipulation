# ros_pointcloud_manipulation

This project enables the pointcloud manipulation funciton including **angular rotations** (roll, pitch, yaw) and **linear translations** (x, y, z) between two frames.

The node subscribe to rostopic: /velodyne_points and publish two new rostopic: /manipulate_points and /transformation_between_frame
	**/velodyne_points** is in sensor_msgs/PointCloud2 type and contains the original pointcloud info
	**/manipulation_points** is in sensor_msgs/PointCloud2 type and contains the transformed pointcloud info
	**/transformation_between_frame** is in geometry_msgs/TransformStamped type and contains the transformation info between two frames

The parameters of angular rotations and linear translations can be set in the launch file. In the example, the transformation between **velodyne frame** and **transform_frame** is described below:
	1. translate along y axis for 5 meters (0, 5, 0)
	2. rotate along z axis for 90 degrees (0, 0, M_PI/2)
