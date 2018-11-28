/*
 * tf_publisher_node.cpp
 *
 *  Created on: 14.08.2015
 *      Author: neunertm
 */

#include <memory>

#include <ros/ros.h>
#include <ros/console.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <rcars_detector/TagArray.h>

#include "kindr/rotations/RotationEigen.hpp"

namespace rot = kindr::rotations::eigen_impl;
typedef rot::RotationQuaternionPD QPD;

std::map<int, bool> tagSeen;
std::map<int, double> tagLastSeen;

double loop_closure_time_threshold;

void printLoopClosureOffset(const rcars_detector::Tag& detectedTag, const rcars_detector::Tag& estimatorTag)
{
	if (detectedTag.id != estimatorTag.id)
	{
		std::cout << "Error, tag ids do not match" << std::endl;
		return;
	}

	std::array<double, 4> reprojectionErrors;
	double averageReprojectionError = 0.0;
	for (size_t i=0; i<detectedTag.corners.size(); i++)
	{
		double dx = detectedTag.corners[i].x - estimatorTag.corners[i].x;
		double dy = detectedTag.corners[i].y - estimatorTag.corners[i].y;
		reprojectionErrors[i] = std::sqrt(dx*dx + dy*dy);
		averageReprojectionError += reprojectionErrors[i]/4.0;
	}
	std::cout << "Reprojection errors: ("
			<<reprojectionErrors[0]<<","
			<<reprojectionErrors[1]<<","
			<<reprojectionErrors[2]<<","
			<<reprojectionErrors[3]<<")"<<std::endl;

	std::cout << "average reprojection error: " << averageReprojectionError << std::endl;

	double dx = detectedTag.pose.position.x - estimatorTag.pose.position.x;
	double dy = detectedTag.pose.position.y - estimatorTag.pose.position.y;
	double dz = detectedTag.pose.position.z - estimatorTag.pose.position.z;
	double totalOffset = std::sqrt(dx*dx + dy*dy + dz*dz);

	std::cout << "Offset position: " << totalOffset << std::endl;

	QPD qVTdetector(detectedTag.pose.orientation.w, detectedTag.pose.orientation.x, detectedTag.pose.orientation.y, detectedTag.pose.orientation.z);
	QPD qVTestimator(estimatorTag.pose.orientation.w, estimatorTag.pose.orientation.x, estimatorTag.pose.orientation.y, estimatorTag.pose.orientation.z);

	std::cout << "Disparity angle: " << qVTdetector.getDisparityAngle(qVTestimator) << std::endl;
}

void callbackTags(const rcars_detector::TagArrayConstPtr& detectedTags, const rcars_detector::TagArrayConstPtr& estimatorTags)
{
	static bool loopClosureDetected = false;

	if (loopClosureDetected) { return; }

	for (size_t i=0; i<detectedTags->tags.size(); i++)
	{
		const int& tagId = detectedTags->tags[i].id;

		bool tagSeenBefore = (tagLastSeen.find(tagId) != tagLastSeen.end());
		double tagNotSeenSince = detectedTags->header.stamp.toSec() - tagLastSeen[tagId];

		if (tagSeenBefore && (tagNotSeenSince > loop_closure_time_threshold))
		{
			std::cout << "Tag with id "<<tagId<<" had not seen for "<<tagNotSeenSince<<" s but now seen again. Assuming this is a loop closure"<<std::endl;

			int tagIndexEst = -1;
			for (size_t j=0; j<estimatorTags->tags.size(); j++)
			{
				if (estimatorTags->tags[j].id == tagId)
				{
					tagIndexEst = j;
					break;
				}
			}

			if (tagIndexEst == -1)
			{
				std::cout << "Error, loop closure tag not found in estimator." << std::endl;
				return;
			}

			printLoopClosureOffset(detectedTags->tags[i], estimatorTags->tags[tagIndexEst]);

			loopClosureDetected = true;
		} else
		{
			tagLastSeen[tagId] = detectedTags->header.stamp.toSec();
		}
	}
}

int main(int argc, char **argv)
{
	ROS_INFO("Launching RCARS loop_closure_evaluation.");

	ros::init(argc, argv, "loop_closure_evaluation");
	ros::NodeHandle nh;
	ros::NodeHandle nhPrivate("~");

	ros::param::param<double>("~loop_closure_time_threshold", loop_closure_time_threshold, 30);

	message_filters::Subscriber<rcars_detector::TagArray> tagsDetectorSub(nh, "detector/tags", 30);
	message_filters::Subscriber<rcars_detector::TagArray> tagsEstimatorSub(nh, "estimator/tagsCameraFrame", 30);

	typedef message_filters::sync_policies::ApproximateTime<rcars_detector::TagArray, rcars_detector::TagArray> SyncPolicy;

	message_filters::Synchronizer<SyncPolicy> syncDetector(SyncPolicy(10), tagsDetectorSub, tagsEstimatorSub);
	syncDetector.registerCallback(boost::bind(&callbackTags, _1, _2));

	ros::spin();

	return 1;
}
