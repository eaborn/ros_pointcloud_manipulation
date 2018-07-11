/*
 * pointcloud_manipulation.cpp
 *
 *  Created on: July 15, 2018
 *      Author: eaborn
 */

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <fstream>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>


class PointCloudManipulation {

public:
	void pl_receiver(const sensor_msgs::PointCloud2ConstPtr& msg) {

		sensor_msgs::PointCloud2 output;

		geometry_msgs::TransformStamped transform;


		tf2::doTransform(msg, output, transform);
		std::cout << msg->header.stamp.sec << std::endl;
		std::cout << msg->header.stamp.nsec << std::endl;

		//output = *msg;


		if (::ros::ok()) {
			pl_pub.publish(output);
		}
	}





	PointCloudManipulation() {
		// Setup subscribers
		pl_sub = node_handle.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, &PointCloudManipulation::pl_receiver, this);
		// Setup publisher
		pl_pub = node_handle.advertise<sensor_msgs::PointCloud2> ("manipulate_points", 1);
	}


private:
	ros::NodeHandle node_handle;
	ros::Subscriber pl_sub;
	ros::Publisher pl_pub;


//	double x_map;
//	double y_map;
//	double heading;

//	Eigen::Matrix<double, 4, 4> transformationP;
//	Eigen::Matrix<double, 4, 1> pointInSensor;
//	Eigen::Matrix<double, 4, 1> pointInVehicle;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "pointcloud_manipulation");
	ros::Time::init();
	PointCloudManipulation pointcloud_manipulation;

	while (ros::ok()) {
		ros::spin();
	}
}
