/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2015 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: cob_perception_common
 * \note
 * ROS package name: cob_object_detection_visualizer
 *
 * \author
 * Author: Richard Bormann
 * \author
 * Supervised by:
 *
 * \date Date of creation: 15.02.2015
 *
 * \brief
 *
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#include <ros/ros.h>

// topics
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//#include <image_transport/image_transport.h>
//#include <image_transport/subscriber_filter.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// opencv
#include <opencv/cv.h>
#include <opencv2/core/version.hpp>
#if CV_MAJOR_VERSION == 2
// do opencv 2 code
#include <opencv/highgui.h>
#elif CV_MAJOR_VERSION == 3
// do opencv 3 code
#include <opencv2/highgui.hpp>
#endif

#include <cv_bridge/cv_bridge.h>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <eigen_conversions/eigen_msg.h>

#include "cob_object_detection_msgs/DetectionArray.h"

#include <map>
#include <vector>
#include <string>
#include <sstream>


class ObjectDetectionVisualizer
{
public:
	ObjectDetectionVisualizer(ros::NodeHandle& nh)
	: node_handle_(nh), prev_marker_array_size_(0), projection_matrix_received_(false), image_counter_(0)
	{
		// parameters
		ros::NodeHandle pnh("~");
		std::cout << "\n========== ObjectDetectionVisualizer Parameters ==========\n";
		pnh.param("display_rviz_markers", display_rviz_markers_, true);
		std::cout << "display_rviz_markers: " << display_rviz_markers_ << std::endl;
		pnh.param("display_detection_image", display_detection_image_, true);
		std::cout << "display_detection_image: " << display_detection_image_ << std::endl;

		projection_matrix_ = cv::Mat::eye(3, 3, CV_64FC1);

		// Rviz visualization
		if (display_rviz_markers_ == true)
		{
			detection_array_sub_ = node_handle_.subscribe("detection_array_topic", 1, &ObjectDetectionVisualizer::objectDetectionArrayCallback, this);
			marker_array_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("object_detection_marker_array", 0);
		}

		// detection image visualization
		if (display_detection_image_ == true)
		{
//			it_ = new image_transport::ImageTransport(node_handle_);
//			color_image_sub_.subscribe(*it_, "color_image", 1);
			pointcloud_sub_ = node_handle_.subscribe("pointcloud", 1, &ObjectDetectionVisualizer::pointcloudCallback, this);
			detection_array_sub_ = node_handle_.subscribe("detection_array_topic", 1, &ObjectDetectionVisualizer::objectDetectionDisplayCallback, this);

//			sync_detection_array_sub_.subscribe(node_handle_, "detection_array_topic", 1);
//			sync_input_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_object_detection_msgs::DetectionArray, sensor_msgs::PointCloud2> >(2);
//			sync_input_->connectInput(sync_detection_array_sub_, pointcloud_sub_);
//			sync_input_->registerCallback(boost::bind(&ObjectDetectionVisualizer::detectionImageCallback, this, _1, _2));

			pointcloud_info_sub_ = node_handle_.subscribe("pointcloud_info", 1, &ObjectDetectionVisualizer::pointcloudInfoCallback, this);
		}
	}

	~ObjectDetectionVisualizer()
	{
//		if (it_ != NULL)
//			delete it_;
//		if (sync_input_ != NULL)
//			delete sync_input_;
	}

private:

	void objectDetectionArrayCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& object_detection_msg)
	{
		// 3 arrows for each coordinate system of each detected fiducial
		const unsigned int detection_array_size = object_detection_msg->detections.size();
		const unsigned int markers_per_detection = 3+1;
		const unsigned int marker_array_size = markers_per_detection * detection_array_size;
		if (marker_array_size >= prev_marker_array_size_) {
			marker_array_msg_.markers.resize(marker_array_size);
		}

		// publish a coordinate system from arrow markers for each object
		for (unsigned int i = 0; i < detection_array_size; ++i)
		{
			for (unsigned int j = 0; j < 3; ++j)
			{
				unsigned int idx = markers_per_detection * i + j;
				marker_array_msg_.markers[idx].header = object_detection_msg->header;
				marker_array_msg_.markers[idx].ns = "object_detection";
				marker_array_msg_.markers[idx].id = idx;
				marker_array_msg_.markers[idx].type = visualization_msgs::Marker::ARROW;
				marker_array_msg_.markers[idx].action = visualization_msgs::Marker::ADD;
				marker_array_msg_.markers[idx].color.a = 0.85;
				marker_array_msg_.markers[idx].color.r = 0;
				marker_array_msg_.markers[idx].color.g = 0;
				marker_array_msg_.markers[idx].color.b = 0;

				marker_array_msg_.markers[idx].points.resize(2);
				marker_array_msg_.markers[idx].points[0].x = 0.0;
				marker_array_msg_.markers[idx].points[0].y = 0.0;
				marker_array_msg_.markers[idx].points[0].z = 0.0;
				marker_array_msg_.markers[idx].points[1].x = 0.0;
				marker_array_msg_.markers[idx].points[1].y = 0.0;
				marker_array_msg_.markers[idx].points[1].z = 0.0;
				if (j == 0)
				{
					marker_array_msg_.markers[idx].points[1].x = 0.2;
					marker_array_msg_.markers[idx].color.r = 255;
				}
				else if (j == 1)
				{
					marker_array_msg_.markers[idx].points[1].y = 0.2;
					marker_array_msg_.markers[idx].color.g = 255;
				}
				else if (j == 2)
				{
					marker_array_msg_.markers[idx].points[1].z = 0.2;
					marker_array_msg_.markers[idx].color.b = 255;
				}

				marker_array_msg_.markers[idx].pose = object_detection_msg->detections[i].pose.pose;

				marker_array_msg_.markers[idx].lifetime = ros::Duration(2);
				marker_array_msg_.markers[idx].scale.x = 0.01; // shaft diameter
				marker_array_msg_.markers[idx].scale.y = 0.015; // head diameter
				marker_array_msg_.markers[idx].scale.z = 0; // head length 0=default
			}
			for (unsigned int j = 3; j < 4; ++j)
			{
				unsigned int idx = markers_per_detection * i + j;
				marker_array_msg_.markers[idx].header = object_detection_msg->header;
				marker_array_msg_.markers[idx].ns = "object_detection";
				marker_array_msg_.markers[idx].id = idx;
				marker_array_msg_.markers[idx].type = visualization_msgs::Marker::CUBE;
				marker_array_msg_.markers[idx].action = visualization_msgs::Marker::ADD;
				marker_array_msg_.markers[idx].color.a = 0.25;
				marker_array_msg_.markers[idx].color.r = 128;
				marker_array_msg_.markers[idx].color.g = 128;
				marker_array_msg_.markers[idx].color.b = 128;

				marker_array_msg_.markers[idx].pose = object_detection_msg->detections[i].pose.pose;

				marker_array_msg_.markers[idx].lifetime = ros::Duration(2);
				marker_array_msg_.markers[idx].scale.x = object_detection_msg->detections[i].bounding_box_lwh.x;
				marker_array_msg_.markers[idx].scale.y = object_detection_msg->detections[i].bounding_box_lwh.y;
				marker_array_msg_.markers[idx].scale.z = object_detection_msg->detections[i].bounding_box_lwh.z;
			}

			if (prev_marker_array_size_ > marker_array_size)
				for (unsigned int i = marker_array_size; i < prev_marker_array_size_; ++i)
					marker_array_msg_.markers[i].action = visualization_msgs::Marker::DELETE;
			prev_marker_array_size_ = marker_array_size;

			marker_array_publisher_.publish(marker_array_msg_);
		}
	}

	bool convertImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
	{
		try
		{
			image_ptr = cv_bridge::toCvShare(image_msg, image_msg->encoding);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("ObjectDetectionVisualizer::convertColorImageMessageToMat: cv_bridge exception: %s", e.what());
			return false;
		}
		image = image_ptr->image;

		return true;
	}

	bool convertPclMessageToMat(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg, pcl::PointCloud<pcl::PointXYZRGB>& pointcloud, cv::Mat& color_image)
	{
		pcl::fromROSMsg(*pointcloud_msg, pointcloud);
		color_image.create(pointcloud.height, pointcloud.width, CV_8UC3);
		for (int v = 0; v < (int)pointcloud.height; v++)
		{
			for (int u = 0; u < (int)pointcloud.width; u++)
			{
				const pcl::PointXYZRGB& point = pointcloud(u, v);
				color_image.at<cv::Vec3b>(v,u) = cv::Vec3b(point.b, point.g, point.r);
			}
		}
		return true;
	}

	void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
	{
		// secure this access with a mutex
		boost::mutex::scoped_lock lock(color_image_mutex_);

		pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
		convertPclMessageToMat(pointcloud_msg, pointcloud, color_image_);
	}

	void objectDetectionDisplayCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& object_detection_msg)
	{
		cv::Mat image;
		{
			// secure this access with a mutex
			boost::mutex::scoped_lock lock(color_image_mutex_);
			image = color_image_.clone();
		}
		renderDetections(image, object_detection_msg);

		cv::Mat display;
		cv::resize(image, display, cv::Size(), 0.5, 0.5);
		cv::imshow("object detections", display);
		cv::waitKey(50);

		std::stringstream file;
		file << "object_detection_visualizer/" << image_counter_ << ".png";
		cv::imwrite(file.str().c_str(), image);
		image_counter_++;
	}

	void detectionImageCallback(const cob_object_detection_msgs::DetectionArray::ConstPtr& object_detection_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
	{
//		// read image
//		cv_bridge::CvImageConstPtr color_image_ptr;
//		cv::Mat color_image;
//		if (convertImageMessageToMat(image_msg, color_image_ptr, color_image) == false)
//			return;

		cv::Mat image;
		pcl::PointCloud<pcl::PointXYZRGB> pointcloud;
		convertPclMessageToMat(pointcloud_msg, pointcloud, image);

		renderDetections(image, object_detection_msg);

		cv::Mat display;
		cv::resize(image, display, cv::Size(), 0.5, 0.5);
		cv::imshow("object detections", display);
		cv::waitKey(50);

		std::stringstream file;
		file << "object_detection_visualizer/" << image_counter_ << ".png";
		cv::imwrite(file.str().c_str(), image);
		image_counter_++;
	}

	bool renderDetections(cv::Mat& image, const cob_object_detection_msgs::DetectionArray::ConstPtr& object_detection_msg)
	{
		if (projection_matrix_received_ == false)
		{
			ROS_WARN("Did not receive a projection matrix yet.");
			return false;
		}

		// render 3-D detection results
		// project bounding box of the object on the image
		if (object_detection_msg->detections.size() <= 0)
			return false;

		for (size_t o=0; o<object_detection_msg->detections.size(); ++o)
		{
			const cob_object_detection_msgs::Detection& detection = object_detection_msg->detections[o];

			// compute 8 corner points of bounding box and transform them into the camera coordinate system
			// reproduce 8 points in a special order (simplifies drawing a box):
			// x y z
			// - - -
			// + - -
			// + + -
			// - + -
			// - - +
			// + - +
			// + + +
			// - + +
			std::cout << "New box corners for object " << detection.label << ": \n";
			Eigen::Affine3d pose;
			tf::poseMsgToEigen(detection.pose.pose, pose);	// i.e. p = the transform pointing from camera to object coordinate system
			std::vector<cv::Vec3d> corners_3d;
			for (int k=0; k<2; ++k)	// z is given in full length, the object center is at the bottom
			{
				int s = 1;
				for (int j=-1; j<2; j+=2)	// x and y are only provided in half side lengths, center is in the object center
				{
					for (int i=-1; i<2; i+=2)
					{
						Eigen::Affine3d corner = pose * Eigen::Translation3d(i*s*detection.bounding_box_lwh.x, j*detection.bounding_box_lwh.y, k*detection.bounding_box_lwh.z);
						corners_3d.push_back(cv::Vec3d(corner.translation()(0), corner.translation()(1), corner.translation()(2)));
						//std::cout << " > " << corner.translation()(0) << ", " << corner.translation()(1) << ", " << corner.translation()(2) << std::endl;
					}
					s *= -1;
				}
			}

			// project xyz coordinates to image coordinates
			std::vector<cv::Point> corners_2d(corners_3d.size());
			for (size_t i=0; i<corners_3d.size(); ++i)
				corners_2d[i] = projectPoint(corners_3d[i]);

			// draw all detected bounding boxes
			if (object_name_to_color_map_.find(detection.label) == object_name_to_color_map_.end())
				object_name_to_color_map_[detection.label] = CV_RGB(rand()%256,rand()%256,rand()%256);
			if (drawDetectedModel3D(image, corners_2d, object_name_to_color_map_[detection.label]) == false )
			{
				ROS_ERROR("Unable to draw detected bounding boxes!");
				return false;
			}

			// annotate bounding box with object name
			cv::Point center_top;
			for (int i=4; i<8; ++i)
				center_top += corners_2d[i];
			center_top *= 0.25;
			cv::putText(image, detection.label, center_top, cv::FONT_HERSHEY_SIMPLEX, 0.75, object_name_to_color_map_[detection.label], 2);
		}

		return true;
	}

	cv::Point projectPoint(const cv::Vec3d& point_3d)
	{
		if (projection_matrix_received_ == false)
		{
			ROS_WARN("Did not receive a projection matrix yet.");
			return cv::Point();
		}
		cv::Mat uvw = projection_matrix_ * cv::Mat(point_3d);
		return cv::Point(uvw.at<double>(0,0)/uvw.at<double>(2,0), uvw.at<double>(1,0)/uvw.at<double>(2,0));
	}

	bool drawDetectedModel3D(cv::Mat image, const std::vector<cv::Point>& corners_2d, const cv::Scalar& color)
	{
		// Draw lower rectangle of bounding box
		for (unsigned int j=0; j<3; ++j)
		{
			cv::line(image, corners_2d[j], corners_2d[j+1], color, 3);
		}
		cv::line(image, corners_2d[3], corners_2d[0], color, 3);

		// Draw upper rectangle of bounding box
		for (unsigned int j=4; j<corners_2d.size()-1; j++)
		{
			cv::line(image, corners_2d[j], corners_2d[j+1], color, 3);
		}
		cv::line(image, corners_2d[7], corners_2d[4], color, 3);

		// Draw side lines of bounding box
		for (unsigned int j=0; j<4; j++)
		{
			cv::line(image, corners_2d[j], corners_2d[j+4], color, 3);
		}

		return true;
	}

	void pointcloudInfoCallback(const sensor_msgs::CameraInfoConstPtr &data)
	{
		double* f_ptr = projection_matrix_.ptr<double>(0);
		for (int i = 0; i < 9; i++)
			f_ptr[i] = data->K[i];

		ROS_INFO("[object_detection_visualizer] Received new projection matrix:");
		std::cout << "\t... / " << std::setw(8) <<  projection_matrix_.at<double>(0, 0) << " ";
		std::cout << std::setw(8) << projection_matrix_.at<double>(0, 1) << " ";
		std::cout << std::setw(8) << projection_matrix_.at<double>(0, 2) << " \\ " << std::endl;
		std::cout << "\t... | " << std::setw(8) << projection_matrix_.at<double>(1, 0) << " ";
		std::cout << std::setw(8) << projection_matrix_.at<double>(1, 1) << " ";
		std::cout << std::setw(8) << projection_matrix_.at<double>(1, 2) << " | "<< std::endl;;
		std::cout << "\t... \\ " << std::setw(8) << projection_matrix_.at<double>(2, 0) << " ";
		std::cout << std::setw(8) << projection_matrix_.at<double>(2, 1) << " ";
		std::cout << std::setw(8) << projection_matrix_.at<double>(2, 2) << " / "<< std::endl << std::endl;

		projection_matrix_received_ = true;
		pointcloud_info_sub_.shutdown();
	}

	ros::NodeHandle node_handle_;
	ros::Subscriber detection_array_sub_;
	ros::Publisher marker_array_publisher_;
	unsigned int prev_marker_array_size_;
	visualization_msgs::MarkerArray marker_array_msg_;

//	image_transport::ImageTransport* it_;
//	image_transport::SubscriberFilter color_image_sub_; ///< color camera image topic
//	message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_; ///< receives the detection messages
//	message_filters::Subscriber<cob_object_detection_msgs::DetectionArray> sync_detection_array_sub_; ///< receives the detection messages
//	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_object_detection_msgs::DetectionArray, sensor_msgs::PointCloud2> >* sync_input_;
	ros::Subscriber pointcloud_info_sub_;
	ros::Subscriber pointcloud_sub_;

	bool projection_matrix_received_;
	cv::Mat projection_matrix_;		// 3x3 intrinsic matrix

	boost::mutex color_image_mutex_; // secures read and write operations on camera data
	cv::Mat color_image_;

	std::map<std::string, cv::Scalar> object_name_to_color_map_;
	int image_counter_;

	// parameters
	bool display_rviz_markers_;
	bool display_detection_image_;
};


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	/// initialize ROS, specify name of node
	ros::init(argc, argv, "object_detection_visualizer");

	/// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	/// Create camera node class instance
	ObjectDetectionVisualizer odv(nh);

//	ros::MultiThreadedSpinner spinner(2); // Use 4 threads
//	spinner.spin();
	ros::spin();

	return 0;
}
