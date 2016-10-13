/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_perception_common
 * ROS package name: cob_image_flip
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Richard Bormann, email:richard.bormann@ipa.fraunhofer.de
 * Supervised by: Richard Bormann, email:richard.bormann@ipa.fraunhofer.de
 *
 * Date of creation: May 2011
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <stereo_msgs/DisparityImage.h>
#include <tf/transform_listener.h>
#include <cob_perception_msgs/Float64ArrayStamped.h>

// topics
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// point cloud
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

// System
#include <iostream>

namespace cob_image_flip
{
class ImageFlip
{
protected:

	// parameters
	int rotation_mode_;				// rotation mode (0=fixed angle as defined  by rotation_angle, 1=automatic upright rotation against gravity using reference coordinate frame with upwards directed z-axis, 2=as 1 but image rotation only in 90deg steps -> faster)
	double rotation_angle_;			// image rotation [in deg] (especially efficient with 0, 90, 180, 270 deg)
	std::string reference_frame_;	// reference world coordinate frame with z-axis pointing upwards (= against gravity direction)
//	std::string camera_frame_;		// camera coordinate frame (image coordinate system with x-axis to the right, y-axis downwards, z-axis into viewing direction of the camera)
	bool flip_color_image_;			// flip color image
	bool flip_pointcloud_;			// flip point cloud (usually unnecessary because tf takes care of this with original point cloud)
	bool flip_disparity_image_;		// flip disparity image
	bool display_warnings_;			// display warning if transformation not available
	bool display_timing_;			// display timing information

	double last_rotation_angle_;
	double last_rotation_factor_;

	// subscriber counters
	unsigned int img_sub_counter_;
	unsigned int pc_sub_counter_;
	unsigned int disparity_sub_counter_;

	ros::Subscriber point_cloud_sub_;	///< point cloud input topic
	ros::Publisher point_cloud_pub_;	///< point cloud output topic
	ros::Publisher point_cloud_2d_transform_pub_;	///< publisher for the transformation matrix for the in plane rotation and translation in the image plane, this matrix converts coordinates of the turned image into coordinates of the original image (with camera calibration applicable there): p_original = T * p_turned
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter color_camera_image_sub_;	///< color camera image input topic
	image_transport::Publisher color_camera_image_pub_;			///< color camera image output topic
	ros::Publisher color_camera_image_2d_transform_pub_;	///< publisher for the transformation matrix for the in plane rotation and translation in the image plane, this matrix converts coordinates of the turned image into coordinates of the original image (with camera calibration applicable there): p_original = T * p_turned
	ros::Subscriber disparity_image_sub_;	///< disparity image input topic
	ros::Publisher disparity_image_pub_;	///< disparity image output topic
	ros::Publisher disparity_image_2d_transform_pub_;	///< publisher for the transformation matrix for the in plane rotation and translation in the image plane, this matrix converts coordinates of the turned image into coordinates of the original image (with camera calibration applicable there): p_original = T * p_turned

	tf::TransformListener transform_listener_;

	ros::NodeHandle node_handle_; ///< ROS node handle

public:

	enum RotationMode { FIXED_ANGLE=0, AUTOMATIC_GRAVITY_DIRECTION=1, AUTOMATIC_GRAVITY_DIRECTION_90=2 };	// (0=fixed angle as defined  by rotation_angle, 1=automatic upright rotation against gravity using reference coordinate frame with upwards directed z-axis, 2=as 1 but image rotation only in 90deg steps -> faster)

	ImageFlip(ros::NodeHandle nh);

	~ImageFlip();

	double determineRotationAngle(const std::string& camera_frame_id, const ros::Time& time);

	bool convertImageMessageToMat(const sensor_msgs::Image::ConstPtr& color_image_msg, cv_bridge::CvImageConstPtr& color_image_ptr, cv::Mat& color_image);//, const sensor_msgs::image_encodings image_encoding = sensor_msgs::image_encodings::TYPE_32FC1);


	void imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg);

	void imgConnectCB(const image_transport::SingleSubscriberPublisher& pub);

	void imgDisconnectCB(const image_transport::SingleSubscriberPublisher& pub);


	void pcCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg);

	void pcConnectCB(const ros::SingleSubscriberPublisher& pub);

	void pcDisconnectCB(const ros::SingleSubscriberPublisher& pub);


	void disparityCallback(const stereo_msgs::DisparityImage::ConstPtr& disparity_image_msg);

	void disparityConnectCB(const ros::SingleSubscriberPublisher& pub);

	void disparityDisconnectCB(const ros::SingleSubscriberPublisher& pub);
};

}
