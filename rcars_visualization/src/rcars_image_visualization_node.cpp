#include <ros/ros.h>
#include <ros/console.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv/cv.hpp>

#include <kindr/rotations/RotationEigen.hpp>
#include <kindr/poses/eigen/HomogeneousTransformation.hpp>

#include "rcars_detector/Tag.h"
#include "rcars_detector/TagArray.h"

#include <sstream>
#include <math.h>

using namespace std;

// namespace aliasing for kindr library
namespace rot = kindr::rotations::eigen_impl;
namespace pose = kindr::poses::eigen_impl;
namespace phys = kindr::phys_quant::eigen_impl;


// publishes debug images with marked detections
image_transport::Publisher detectorPublisher;
image_transport::Publisher estimatorPublisher;


// ROS parameters
bool showReprojection = false;


/** \brief Visulaizes the detections by marking corners and printing the tag id in the image
 * \param detections Detection information containing information such as tagIDs, corners of detected tags
 * \param image Reference to output image to draw on
 * \param Header information from the tag to copy the time stamp from
 * */
void visualizeDetections(const rcars_detector::TagArrayConstPtr& detectedTags, cv::Mat& image, bool drawId = false, bool useRectangle = false)
{
	// iterate over detections
	for(size_t i=0; i<detectedTags->tags.size(); ++i)
	{
		std::vector<geometry_msgs::Point> corners;
		for (size_t j=0; j<4; j++)
		{
			corners.push_back(detectedTags->tags[i].corners[j]);
		}

		// calculate distance between corners
		int dx = corners[0].x-corners[2].x;
		int dy = corners[0].y-corners[2].y;

		// set the disparity according to diagonal
		int disparity = sqrt(dx*dx + dy*dy);

		// set corner circle size, thickness and tag id font size depending on disparity
		int circleSize = disparity/15;
		int circleThickness = sqrt(disparity/30);
		double fontSize = disparity/100.0;

		// threshholding
		if (circleSize < 5) { circleSize = 5; }
		if (circleThickness < 2) { circleThickness = 2; }
		if (fontSize < 0.5) { fontSize = 0.5; }
		if (fontSize > 4.0) { fontSize = 4.0; }

		if (useRectangle)
		{
			for (size_t i=0; i<4; i++)
			{
				int rectangleSize = 2.0*circleSize;
				cv::Point topLeft(cv::Point(corners[i].x-rectangleSize/2.0, corners[i].y-rectangleSize/2.0));
				cv::Point bottomRight(cv::Point(corners[i].x+rectangleSize/2.0, corners[i].y+rectangleSize/2.0));
				cv::rectangle(image, topLeft, bottomRight, CV_RGB(0,0,255), circleThickness);
			}
		} else
		{
			// Draw circles at corners of detected tags on the video stream
			for (size_t i=0; i<4; i++)
			{
				cv::circle(image, cv::Point(corners[i].x, corners[i].y), circleSize, CV_RGB(255,0,0), circleThickness);
			}
		}

		if (drawId)
		{
			// Draw tag id
			cv::putText(
					image,
					std::to_string(detectedTags->tags[i].id),
					cv::Point(0.5*(corners[0].x+corners[3].x), 0.5*(corners[0].y+corners[3].y)),
					cv::FONT_HERSHEY_SIMPLEX,
					fontSize,
					CV_RGB(0,255,0),
					2
			);
		}
	}
}


void callbackDetector(const sensor_msgs::ImageConstPtr& image, const rcars_detector::TagArrayConstPtr& detectedTags)
{
	cv_bridge::CvImageConstPtr cvPtrGray;

	// convert incoming image to OpenCV
	try
	{
		// pointer to gray-scale image
		cvPtrGray = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// create a new image pointer
	cv_bridge::CvImagePtr cvImage(new cv_bridge::CvImage);

	// set gray-scale and time
	cvImage->encoding = "rgb8";
	cvImage->header = image->header;

	// get reference to OpenCV implementation
	cv::Mat& colorImage = cvImage->image;
	cv::cvtColor(cvPtrGray->image, colorImage, CV_GRAY2RGB);

	// visualize the detections
	visualizeDetections(detectedTags, colorImage, true, false);

    // Publish Image
	detectorPublisher.publish(cvImage->toImageMsg());
}

void callbackFull(const sensor_msgs::ImageConstPtr& image, const rcars_detector::TagArrayConstPtr& detectedTags, const rcars_detector::TagArrayConstPtr& estimatedTags)
{
	cv_bridge::CvImageConstPtr cvPtrGray;

	// convert incoming image to OpenCV
	try
	{
		cvPtrGray = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::MONO8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// create a new image pointer
	cv_bridge::CvImagePtr cvImage(new cv_bridge::CvImage);

	// set gray-scale and time
	cvImage->encoding = "rgb8";
	cvImage->header = image->header;

	// get reference to OpenCV implementation
	cv::Mat& colorImage = cvImage->image;
	cv::cvtColor(cvPtrGray->image, colorImage, CV_GRAY2RGB);

	// visualize the detections
	visualizeDetections(detectedTags, colorImage, true);

	// remove tags that are not seen by the detector
	rcars_detector::TagArrayPtr estimatedTagsClean(new rcars_detector::TagArray);
	estimatedTagsClean->header = estimatedTags->header;
	for (size_t i=0; i<estimatedTags->tags.size(); i++)
	{
		for (size_t j=0; j<detectedTags->tags.size(); j++)
		{
			if (detectedTags->tags[j].id == estimatedTags->tags[i].id)
			{
				estimatedTagsClean->tags.push_back(estimatedTags->tags[i]);
				break;
			}
		}
	}
	visualizeDetections(estimatedTagsClean, colorImage, false, true);

    // Publish Image
	estimatorPublisher.publish(cvImage->toImageMsg());
}


/** \brief Main function
 * */

int main(int argc, char **argv)
{
	ROS_INFO("Launching RCARS visualization.");

	ros::init(argc, argv, "visualizer");
	ros::NodeHandle nh;
	ros::NodeHandle nhPrivate("~");
	image_transport::ImageTransport it(nhPrivate);

	// publisher for image with marked corners (for debugging)
	std::string camTopicName;
	std::string extNodeName;

	ros::param::param<std::string>("~extNodeName", extNodeName, "");
	ros::param::param<std::string>("~camTopicName", camTopicName, "/cam0");

	detectorPublisher = it.advertise("detectorImage", 1);
	estimatorPublisher = it.advertise("detectorEstimatorImage", 1);

	message_filters::Subscriber<sensor_msgs::Image> imageSub(nh, extNodeName + camTopicName + "/image_rect", 2);
	message_filters::Subscriber<rcars_detector::TagArray> detectorSub(nh, "detector/tags", 2);
	message_filters::Subscriber<rcars_detector::TagArray> estimatorSub(nh, "estimator/tagsCameraFrame", 2);

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, rcars_detector::TagArray> DetectorSyncPolicy;
	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, rcars_detector::TagArray, rcars_detector::TagArray> FullSyncPolicy;

	message_filters::Synchronizer<DetectorSyncPolicy> syncDetector(DetectorSyncPolicy(10), imageSub, detectorSub);
	syncDetector.registerCallback(boost::bind(&callbackDetector, _1, _2));

	message_filters::Synchronizer<FullSyncPolicy> syncFull(FullSyncPolicy(10), imageSub, detectorSub, estimatorSub);
	syncFull.registerCallback(boost::bind(&callbackFull, _1, _2, _3));

	ros::spin();

	// everything hopefully went well
	return 0;
}
