/*
 * RcarsToRobotPose.h
 *
 * Created on: 26.09.2016
 * 	   Author: mgiftthaler<mgiftthaler@ethz.ch> 
 * 
 * This class picks the relative transform between the first tag that is seen at startup and the RCARS frame
 * and sets it as a reference.
 * It outputs all future pose as a relative pose to the first reference, therefore the robot starts at [0 0 0 ... 0]
 *
 */

#ifndef RCARS_TO_ROBOT_POSE_H_
#define RCARS_TO_ROBOT_POSE_H_

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>


#include <kindr/poses/eigen/HomogeneousTransformation.hpp>
#include <kindr/rotations/eigen/RotationQuaternion.hpp>
#include <kindr/rotations/eigen/EulerAnglesXyz.hpp>

#include <rcars_detector/TagArray.h>

typedef kindr::poses::eigen_impl::HomogeneousTransformationPosition3RotationQuaternionD Pose;
typedef kindr::phys_quant::eigen_impl::Position3D Pos3d;
typedef kindr::rotations::eigen_impl::RotationQuaternionPD Quat;
typedef kindr::rotations::eigen_impl::EulerAnglesXyzPD EulerXyz;


class RcarsToRobotPose{

public:
	RcarsToRobotPose(ros::NodeHandle& nh, const Eigen::Vector3d& robotFrameSensorOffset = Eigen::Vector3d::Zero(), const size_t averageSamples = 10)
:	nh_(nh),
 	initialized_(false)
{
		// offset of the camera sensor from robot center
		C_x_eigen = robotFrameSensorOffset;

		numSamplesAveragedForInitPose = averageSamples;

		PosePub3D_ =nh.advertise<geometry_msgs::PoseStamped>("/FullBasePose", 2);

		// subscribe to RCARS
		tagPoseSub_ = nh_.subscribe("/rcars/estimator/tagsCameraFrame", 2, &RcarsToRobotPose::tagsCameraFrameCallback, this, ros::TransportHints().tcpNoDelay());

}

private:

	void tagsCameraFrameCallback(const rcars_detector::TagArray& msg){

		// frames
		// C = current
		// T = tag
		// I = initial
		// P = pamm

		if (msg.tags.size() == 0) return;

		auto& tagPose = msg.tags[0].pose;	// vector camera to tag expressed in camera frame

		if(!initialized_)
		{
			initId_  = msg.tags[0].id;

			Pose newPoseIn = messagePoseToKindr(tagPose);

			// collecting samples for init pose trafo averaging
			if(eulerAnglesStock.size() < numSamplesAveragedForInitPose){
				EulerXyz temp ((newPoseIn.getRotation()).getUnique());
				eulerAnglesStock.push_back(temp);
				positionStock.push_back(newPoseIn.getPosition());
			}
			else if (eulerAnglesStock.size() == numSamplesAveragedForInitPose){
				// average euler angles
				double alpha_avg, beta_avg, gamma_avg;
				double alpha_sum = 0.0;
				double beta_sum = 0.0;
				double gamma_sum = 0.0;
				Pos3d pos_sum (0, 0, 0);
				Pos3d pos_avg;

				for(size_t i = 0; i < eulerAnglesStock.size(); i++){
					alpha_sum += eulerAnglesStock[i].toImplementation()(0);
					beta_sum  += eulerAnglesStock[i].toImplementation()(1);
					gamma_sum += eulerAnglesStock[i].toImplementation()(2);
					pos_sum += positionStock[i];
				}

				alpha_avg = alpha_sum / eulerAnglesStock.size();
				beta_avg  = beta_sum / eulerAnglesStock.size();
				gamma_avg = gamma_sum / eulerAnglesStock.size();
				EulerXyz eulerAveraged (alpha_avg, beta_avg, gamma_avg);
				pos_avg = pos_sum / positionStock.size();

				q_TI_.getPosition() = pos_avg;

				Quat ttemp (eulerAveraged);
				q_TI_.getRotation() = ttemp;

				initialized_ = true;
			}
			else
				throw std::runtime_error("initialization of rcars to basepose failed");


			return;
		}

		if(initId_ !=  msg.tags[0].id)
			throw std::runtime_error("TAG id's of first tag in message and the originally selected ref-frame not matching.");


		Pose q_TC_ = messagePoseToKindr(tagPose);

		Pos3d C_x(C_x_eigen);

		// T_x = q_TC * C_x
		Pos3d T_x = q_TC_.transform(C_x);


		// I_x = q_IT * T_x
		Pos3d I_x = q_TI_.inverseTransform(T_x);

		Quat q_CT = q_TC_.getRotation().inverted();

		//q_CI = q_IT * q_CT
		Quat q_CI = q_CT * q_TI_.getRotation();

		// transcribe to eigen types and publish for reference
		Eigen::Vector3d I_x_eigen 	= I_x.toImplementation();
		Eigen::Quaterniond q_CI_eigen = q_CI.toImplementation();
		publishFullPose(I_x_eigen, q_CI_eigen);

	}


	void publishFullPose(const Eigen::Vector3d& I_x_eigen, const Eigen::Quaterniond& q_CI_eigen){

		geometry_msgs::PoseStamped testMsg;
		testMsg.header.frame_id = "/world";
		testMsg.pose.position.x = I_x_eigen(0);
		testMsg.pose.position.y = I_x_eigen(1);
		testMsg.pose.position.z = I_x_eigen(2);
		testMsg.pose.orientation.w = q_CI_eigen.w();
		testMsg.pose.orientation.x = q_CI_eigen.x();
		testMsg.pose.orientation.y = q_CI_eigen.y();
		testMsg.pose.orientation.z = q_CI_eigen.z();
		PosePub3D_.publish(testMsg);
	}


	Pose messagePoseToKindr(geometry_msgs::Pose msgPose){

		auto& pos = msgPose.position;
		auto& quat = msgPose.orientation;

		Quat q_CP(EulerXyz(M_PI/2.0, 0.0, M_PI/2));
		q_CP.invert();

		Pos3d position(-pos.x,-pos.y,-pos.z);
		Quat quatEigen(quat.w, quat.x, quat.y, quat.z);
		Pose pose(quatEigen.rotate(position), quatEigen);

		pose.getRotation() = pose.getRotation()*q_CP;
		return pose;
	}



	ros::NodeHandle nh_;
	ros::Publisher PosePub3D_;	// for publishing the full 3D pose rotated into the robot frame

	ros::Subscriber tagPoseSub_;	// subscribe to r'cars message


	size_t initId_;			// the id of the first frame listed in the message
	Pose q_TI_;

	bool initialized_;

	Eigen::Vector3d C_x_eigen; // offset of the VI sensor from the robot center

	std::vector<EulerXyz> 	eulerAnglesStock; // to average init pose
	std::vector<Pos3d> 		positionStock;
	size_t 					numSamplesAveragedForInitPose;
};




#endif /* RCARSTOROBOTPOSE_H_ */
