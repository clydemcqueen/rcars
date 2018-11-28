/*
 * rcarsToRobotPoseExample.cpp
 *
 * Created on: 26.09.2016
 * 	   Author: mgiftthaler<mgiftthaler@ethz.ch> 
 * 
 *
 * 	Create an instance of RcarsToRobotPose and spin
 */


#include <ros/ros.h>
#include <quickstart/RcarsToRobotPose.h>


int main (int argc, char* argv[])
{
	ros::init(argc, argv, "RcarsToRobotPose");
	ros::NodeHandle nodeHandle("RcarsToRobotPose");

	// Example: the robot center frame is offset the following values from the camera frame
	Eigen::Vector3d robotFrameSensorOffset;
	robotFrameSensorOffset << -0.33, 0.0, -0.064;

	RcarsToRobotPose rcarsToRobotPose (nodeHandle, robotFrameSensorOffset);

	ros::spin();

}
