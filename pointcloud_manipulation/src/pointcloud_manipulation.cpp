/*
 * pointcloud_manipulation.cpp
 *
 *  Created on: July 13, 2018
 *      Author: eaborn
 */

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>


class PointCloudManipulation {

public:
	void pl_transform(const sensor_msgs::PointCloud2ConstPtr& msg) {

		static tf::TransformBroadcaster br;
		tf::Transform transform;

		//initialize the transformation parameters
		// x = 0;
		// y = 5;
		// z = 0;
		// roll = 0;
		// pitch = 0;
		// yaw = M_PI/2;

		// get parameters from launch file
		node_handle.getParam("/pointcloud_manipulation/x", x);
		node_handle.getParam("/pointcloud_manipulation/y", y);
		node_handle.getParam("/pointcloud_manipulation/z", z);
		node_handle.getParam("/pointcloud_manipulation/roll", roll);
		node_handle.getParam("/pointcloud_manipulation/pitch", pitch);
		node_handle.getParam("/pointcloud_manipulation/yaw", yaw);

		//set up transform
		transform.setOrigin(tf::Vector3(x, y, z));
		tf::Quaternion q;
		q.setRPY(roll, pitch, yaw);
		transform.setRotation(q);

		//frame /transform_frame is the parent frame, frame /velodyne is the child frame. under this transformation, the /velodyne fram is 
		// 5 meters offset to the left from the /transform_frame, then rotate by z axis by 90 degree. (yaw angle)
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "transform_frame", msg->header.frame_id));

		// look up the transform from source frame /velodyne to target frame /transform_frame
		try { 
			listener.waitForTransform("transform_frame",msg->header.frame_id,ros::Time(0),ros::Duration(3.0));
	    	listener.lookupTransform("transform_frame", msg->header.frame_id, ros::Time(0), tfTransform);
	    }
	    //log the error
    	catch(tf::TransformException &exception) { 
      		ROS_ERROR("%s", exception.what());
    	}

		// fill in transformation topic 
		transformation.header = msg->header;
		transformation.child_frame_id = "transform_frame";
	    transformation.transform.translation.x = tfTransform.getOrigin().x();
	    transformation.transform.translation.y = tfTransform.getOrigin().y();
	    transformation.transform.translation.z = tfTransform.getOrigin().z();

	    transformation.transform.rotation.x = tfTransform.getRotation().getAxis().x();
	    transformation.transform.rotation.y = tfTransform.getRotation().getAxis().y();
	    transformation.transform.rotation.z = tfTransform.getRotation().getAxis().z();
	    transformation.transform.rotation.w = tfTransform.getRotation().getW();

		//using pcl_ros::transformPointCloud to transform the pointcloud to target frame
		pcl_ros::transformPointCloud("transform_frame", transform, *msg, output);

		// if you just want to change the whole pointcloud into a different frame, simply change the frame_id to the new frame,
		//the whole pointcloud will be set into the new frame
		//output = *msg;
		//output.header.frame_id = "transform_frame";

		// publish rostopics	
		if (::ros::ok()) {
			pl_pub.publish(output);
			transform_pub.publish(transformation);
		}
	}

	PointCloudManipulation() {
		// Setup subscribers
		pl_sub = node_handle.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 10, &PointCloudManipulation::pl_transform, this);
		// Setup publisher
		pl_pub = node_handle.advertise<sensor_msgs::PointCloud2>("/manipulate_points", 1);
		transform_pub = node_handle.advertise<geometry_msgs::TransformStamped>("/transform_between_frame", 1);
	}

private:
	ros::NodeHandle node_handle;
	ros::Subscriber pl_sub;
	ros::Publisher pl_pub;
	ros::Publisher transform_pub;

	tf::StampedTransform tfTransform;
	tf::TransformListener listener;

	// message to publish
	sensor_msgs::PointCloud2 output;
	geometry_msgs::TransformStamped transformation;

	// declear transformation parameters
	double x;
	double y;
	double z;
	double roll;
	double pitch;
	double yaw;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "pointcloud_manipulation");
	ros::Time::init();
	PointCloudManipulation pointcloud_manipulation;

	while (ros::ok()) {
		ros::spin();
	}
}