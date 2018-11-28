/*
 * rcars_path_visualization_node.cpp
 *
 *  Created on: 06.09.2015
 *      Author: neunertm
 */

#include <ros/ros.h>
#include <ros/console.h>

#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#include "rcars_detector/Tag.h"
#include "rcars_detector/TagArray.h"

int frameSkip = 0;
double lineWidth = 1;
double lineR;
double lineG;
double lineB;
std::string drawFrame;

ros::Publisher linePub;

visualization_msgs::Marker marker;

void setupMessage()
{
	marker.header.frame_id = drawFrame;
	marker.ns = "rcars_path";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = lineWidth;
	marker.scale.y = 1.0;
	marker.scale.z = 1.0;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = lineR;
	marker.color.g = lineG;
	marker.color.b = lineB;
}

void publishPath(const nav_msgs::OdometryConstPtr& pose)
{
	marker.header = pose->header;

	geometry_msgs::Point point = pose->pose.pose.position;
	marker.points.push_back(point);

	linePub.publish(marker);
}

void poseCallback(const nav_msgs::OdometryConstPtr& pose)
{
	static size_t framesSkipped = 99999;
	static double travelledDistance = 0;

	if (framesSkipped >= frameSkip)
	{
		publishPath(pose);

		size_t npoints = marker.points.size();

		if (npoints > 10)
		{

			double dx = marker.points[npoints-1].x - marker.points[npoints-2].x;
			double dy = marker.points[npoints-1].y - marker.points[npoints-2].y;
			double dz = marker.points[npoints-1].z - marker.points[npoints-2].z;

			travelledDistance += std::sqrt(dx*dx + dy*dy + dz*dz);

			std::cout << "Travelled distance: "<< travelledDistance << std::endl;
		}

		framesSkipped = 0;
	} else
	{
		framesSkipped++;
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "path_visualization");
	ros::NodeHandle nh;
	ros::NodeHandle nhPrivate("~");

	linePub = nhPrivate.advertise<visualization_msgs::Marker>("path", 1);

	ros::param::param<int>("~frameSkip", frameSkip, 0);
	ros::param::param<double>("~lineWidth", lineWidth, 1);
	ros::param::param<double>("~lineColorR", lineR, 1);
	ros::param::param<double>("~lineColorG", lineG, 1);
	ros::param::param<double>("~lineColorB", lineB, 1);
	ros::param::param<std::string>("~drawFrame", drawFrame, "world");

	ROS_INFO("Started path visualizer with frame skipe %d, lineWidth %f, and rgb (%f,%f,%f)", frameSkip, lineWidth, lineR, lineG, lineB);

	setupMessage();

	ros::Subscriber sub = nh.subscribe("estimator/filterPose", 2, poseCallback);

	ros::spin();

	return 1;
}
