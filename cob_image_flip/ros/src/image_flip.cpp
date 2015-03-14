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

#include <cob_image_flip/image_flip.h>
#include <geometry_msgs/TransformStamped.h>

namespace cob_image_flip
{
ImageFlip::ImageFlip(ros::NodeHandle nh)
: 	node_handle_(nh), img_sub_counter_(0), pc_sub_counter_(0), transform_listener_(nh), it_(0), last_rotation_angle_(0)
{
	// set parameters
	ROS_DEBUG_STREAM("\n--------------------------\nImage Flip Parameters:\n--------------------------");
	node_handle_.param("rotation_mode", rotation_mode_, 0);
	ROS_DEBUG_STREAM("rotation_mode = " << rotation_mode_);
	if (rotation_mode_ == FIXED_ANGLE)
	{
		// fixed rotation angle
		node_handle_.param("rotation_angle", rotation_angle_, 0.);
		ROS_DEBUG_STREAM("rotation_angle = " << rotation_angle_);
	}
	else if (rotation_mode_ == AUTOMATIC_GRAVITY_DIRECTION || rotation_mode_ == AUTOMATIC_GRAVITY_DIRECTION_90)
	{
		// automatic rotation in gravity direction
		node_handle_.param("reference_frame", reference_frame_, std::string(""));
		ROS_DEBUG_STREAM("reference_frame = " << reference_frame_);
		node_handle_.param("camera_frame", camera_frame_, std::string(""));
		ROS_DEBUG_STREAM("camera_frame = " << camera_frame_);
	}
	else
	{
		ROS_ERROR("Unsupported value for parameter 'rotation_mode'. Exiting the cob_image_flip.");
		exit(1);
	}
	node_handle_.param("flip_color_image", flip_color_image_, false);
	ROS_DEBUG_STREAM("flip_color_image = " << flip_color_image_);
	node_handle_.param("flip_pointcloud", flip_pointcloud_, false);
	ROS_DEBUG_STREAM("flip_pointcloud = " << flip_pointcloud_);
	node_handle_.param<std::string>("pointcloud_data_format", pointcloud_data_format_, "xyz");
	ROS_DEBUG_STREAM("pointcloud_data_format = " << pointcloud_data_format_);
	node_handle_.param("display_warnings", display_warnings_, false);
	ROS_DEBUG_STREAM("display_warnings = " << display_warnings_);

	if (flip_color_image_ == true)
	{
		it_ = new image_transport::ImageTransport(node_handle_);
		color_camera_image_sub_.registerCallback(boost::bind(&ImageFlip::imageCallback, this, _1));
		color_camera_image_pub_ = it_->advertise("colorimage_out", 1, boost::bind(&ImageFlip::imgConnectCB, this, _1), boost::bind(&ImageFlip::imgDisconnectCB, this, _1));
	}

	// point cloud flip
	if (flip_pointcloud_ == true)
	{
		point_cloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("pointcloud_out", 1,  boost::bind(&ImageFlip::pcConnectCB, this, _1), boost::bind(&ImageFlip::pcDisconnectCB, this, _1));
	}

	ROS_DEBUG_STREAM("ImageFlip initialized.");
}

ImageFlip::~ImageFlip()
{
	if (it_ != 0)
		delete it_;
}
bool ImageFlip::convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& color_image_msg, cv_bridge::CvImageConstPtr& color_image_ptr, cv::Mat& color_image)
{
	try
	{
		color_image_ptr = cv_bridge::toCvShare(color_image_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ImageFlip::convertColorImageMessageToMat: cv_bridge exception: %s", e.what());
		return false;
	}
	color_image = color_image_ptr->image;

	return true;
}


void ImageFlip::imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg)
{
	// read image
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;
	if (convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image) == false)
		return;
	cv::Mat color_image_turned;

	double rotation_angle = 0.;
	if (rotation_mode_ == FIXED_ANGLE)
	{
		rotation_angle = rotation_angle_;
	}
	else if (rotation_mode_ == AUTOMATIC_GRAVITY_DIRECTION || rotation_mode_ == AUTOMATIC_GRAVITY_DIRECTION_90)
	{
		// check camera link orientation and decide whether image must be turned around
		try
		{
			// compute angle of camera x-axis against x-y world plane (i.e. in reference coordinates)
			tf::Stamped<tf::Vector3> x_axis_camera(tf::Vector3(1, 0, 0), ros::Time::now(), camera_frame_), x_axis_ref;
			tf::Stamped<tf::Vector3> y_axis_camera(tf::Vector3(0, 1, 0), x_axis_camera.stamp_, camera_frame_), y_axis_ref;
			transform_listener_.transformVector(reference_frame_, x_axis_camera, x_axis_ref);
			transform_listener_.transformVector(reference_frame_, y_axis_camera, y_axis_ref);
 			int factor = (y_axis_ref.z()<0. ? 1 : -1);
			rotation_angle = 180./CV_PI * atan2(-x_axis_ref.z(), factor*sqrt(x_axis_ref.x()*x_axis_ref.x() + x_axis_ref.y()*x_axis_ref.y()));
			if (rotation_mode_ == AUTOMATIC_GRAVITY_DIRECTION_90)
				rotation_angle = 90. * cvRound(rotation_angle*1./90.);
			last_rotation_angle_ = rotation_angle;

//			tf::StampedTransform transform;
//			transform_listener_.lookupTransform(reference_frame_, camera_frame_, ros::Time(0), transform);
//			tfScalar roll, pitch, yaw;
//			transform.getBasis().getRPY(roll, pitch, yaw, 1);
//			std::cout << "xyz: " << transform.getOrigin().getX() << " " << transform.getOrigin().getY() << " " << transform.getOrigin().getZ() << "\n";
//			std::cout << "abcw: " << transform.getRotation().getX() << " " << transform.getRotation().getY() << " " << transform.getRotation().getZ() << " " << transform.getRotation().getW() << "\n";
		}
		catch (tf2::TransformException& ex)
		{
			if (display_warnings_ == true)
				ROS_WARN("%s",ex.what());
			rotation_angle = last_rotation_angle_;
		}
	}
	else
	{
		if (display_warnings_)
			ROS_WARN("ImageFlip::imageCallback: Unsupported rotation mode.");
	}

	// rotate
	// fast hard coded rotations
	if (rotation_angle==0. || rotation_angle==360. || rotation_angle==-360.)
	{
		color_image_turned = color_image;
	}
	else if (rotation_angle==90. || rotation_angle==-270.)
	{
		// rotate images by 90 degrees
		color_image_turned.create(color_image.cols, color_image.rows, color_image.type());
		if (color_image.type() != CV_8UC3)
		{
			ROS_ERROR("ImageFlip::imageCallback: Error: The image format of the color image is not CV_8UC3.\n");
			return;
		}
		for (int v = 0; v < color_image_turned.rows; v++)
			for (int u = 0; u < color_image_turned.cols; u++)
				color_image_turned.at<cv::Vec3b>(v,u) = color_image.at<cv::Vec3b>(color_image.rows-1-u,v);
	}
	else if (rotation_angle==270. || rotation_angle==-90.)
	{
		// rotate images by 270 degrees
		color_image_turned.create(color_image.cols, color_image.rows, color_image.type());
		if (color_image.type() != CV_8UC3)
		{
			std::cout << "ImageFlip::imageCallback: Error: The image format of the color image is not CV_8UC3.\n";
			return;
		}
		for (int v = 0; v < color_image_turned.rows; v++)
			for (int u = 0; u < color_image_turned.cols; u++)
				color_image_turned.at<cv::Vec3b>(v,u) = color_image.at<cv::Vec3b>(u,color_image.cols-1-v);
	}
	else if (rotation_angle==180 || rotation_angle==-180)
	{
		// rotate images by 180 degrees
		color_image_turned.create(color_image.rows, color_image.cols, color_image.type());
		if (color_image.type() != CV_8UC3)
		{
			std::cout << "ImageFlip::imageCallback: Error: The image format of the color image is not CV_8UC3.\n";
			return;
		}
		for (int v = 0; v < color_image.rows; v++)
		{
			uchar* src = color_image.ptr(v);
			uchar* dst = color_image_turned.ptr(color_image.rows - v - 1) + 3 * (color_image.cols - 1);
			for (int u = 0; u < color_image.cols; u++)
			{
				for (int k = 0; k < 3; k++)
				{
					*dst = *src;
					src++;
					dst++;
				}
				dst -= 6;
			}
		}
	}
	else
	{
		// arbitrary rotation
		bool switch_aspect_ratio = !(fabs(sin(rotation_angle*CV_PI/180.)) < 0.707106781);
		if (switch_aspect_ratio==false)
			color_image_turned.create(color_image.rows, color_image.cols, color_image.type());		// automatically decide for landscape or portrait orientation of resulting image
		else
			color_image_turned.create(color_image.cols, color_image.rows, color_image.type());
		if (color_image.type() != CV_8UC3)
		{
			ROS_ERROR("ImageFlip::imageCallback: Error: The image format of the color image is not CV_8UC3.\n");
			return;
		}

		cv::Point center = cv::Point(color_image.cols/2, color_image.rows/2);
		cv::Mat rot_mat = cv::getRotationMatrix2D(center, -rotation_angle, 1.0);
		if (switch_aspect_ratio==true)
		{
			rot_mat.at<double>(0,2) += 0.5*(color_image_turned.cols-color_image.cols);
			rot_mat.at<double>(1,2) += 0.5*(color_image_turned.rows-color_image.rows);
		}
		cv::warpAffine(color_image, color_image_turned, rot_mat, color_image_turned.size());
	}

	// publish turned image
	cv_bridge::CvImage cv_ptr;
	cv_ptr.image = color_image_turned;
	cv_ptr.encoding = "bgr8";
	sensor_msgs::Image::Ptr color_image_turned_msg = cv_ptr.toImageMsg();
	color_image_turned_msg->header = color_image_msg->header;
	color_camera_image_pub_.publish(color_image_turned_msg);
}

void ImageFlip::imgConnectCB(const image_transport::SingleSubscriberPublisher& pub)
{
	img_sub_counter_++;
	if (img_sub_counter_ == 1)
	{
		ROS_DEBUG("ImageFlip::imgConnectCB: Connecting image callback.");
		color_camera_image_sub_.subscribe(*it_, "colorimage_in", 1);
	}
}

void ImageFlip::imgDisconnectCB(const image_transport::SingleSubscriberPublisher& pub)
{
	img_sub_counter_--;
	if (img_sub_counter_ == 0)
	{
		ROS_DEBUG("ImageFlip::imgDisconnectCB: Disconnecting image callback.");
		color_camera_image_sub_.unsubscribe();
	}
}


template <typename T>
void ImageFlip::pcCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
	ROS_WARN("Pointcloud rotation is currently not implemented.");
//	// check camera link orientation and decide whether image must be turned around
//	bool turnAround = false;
//	tf::StampedTransform transform;
//	try
//	{
//		transform_listener_->lookupTransform("/base_link", "/head_cam3d_link", ros::Time(0), transform);
//		tfScalar roll, pitch, yaw;
//		transform.getBasis().getRPY(roll, pitch, yaw, 1);
//		if (roll > 0.0)
//			turnAround = true;
//		//      std::cout << "xyz: " << transform.getOrigin().getX() << " " << transform.getOrigin().getY() << " " << transform.getOrigin().getZ() << "\n";
//		//      std::cout << "abcw: " << transform.getRotation().getX() << " " << transform.getRotation().getY() << " " << transform.getRotation().getZ() << " " << transform.getRotation().getW() << "\n";
//		//      std::cout << "rpy: " << roll << " " << pitch << " " << yaw << "\n";
//	} catch (tf::TransformException ex)
//	{
//		if (display_warnings_ == true)
//			ROS_WARN("%s",ex.what());
//	}
//
//	if (turnAround == false)
//	{
//		// image upright
//		//sensor_msgs::Image color_image_turned_msg = *color_image_msg;
//		//color_image_turned_msg.header.stamp = ros::Time::now();
//		//color_camera_image_pub_.publish(color_image_turned_msg);
//		//sensor_msgs::PointCloud2::ConstPtr point_cloud_turned_msg = point_cloud_msg;
//		point_cloud_pub_.publish(point_cloud_msg);
//	}
//	else
//	{
//		// image upside down
//		// point cloud
//		pcl::PointCloud<T> point_cloud_src;
//		pcl::fromROSMsg(*point_cloud_msg, point_cloud_src);
//
//		boost::shared_ptr<pcl::PointCloud<T> > point_cloud_turned(new pcl::PointCloud<T>);
//		//point_cloud_turned->header = point_cloud_msg->header;
//		point_cloud_turned->header = pcl_conversions::toPCL(point_cloud_msg->header);
//		point_cloud_turned->height = point_cloud_msg->height;
//		point_cloud_turned->width = point_cloud_msg->width;
//		//point_cloud_turned->sensor_orientation_ = point_cloud_msg->sensor_orientation_;
//		//point_cloud_turned->sensor_origin_ = point_cloud_msg->sensor_origin_;
//		point_cloud_turned->is_dense = true;	//point_cloud_msg->is_dense;
//		point_cloud_turned->resize(point_cloud_src.height*point_cloud_src.width);
//		for (int v = (int)point_cloud_src.height - 1; v >= 0; v--)
//		{
//			for (int u = (int)point_cloud_src.width - 1; u >= 0; u--)
//			{
//				(*point_cloud_turned)(point_cloud_src.width-1 - u, point_cloud_src.height-1 - v) = point_cloud_src(u, v);
//			}
//		}
//
//		// publish turned data
//		sensor_msgs::PointCloud2::Ptr point_cloud_turned_msg(new sensor_msgs::PointCloud2);
//		pcl::toROSMsg(*point_cloud_turned, *point_cloud_turned_msg);
//		//point_cloud_turned_msg->header.stamp = ros::Time::now();
//		point_cloud_pub_.publish(point_cloud_turned_msg);
//
//		//      cv::namedWindow("test");
//		//      cv::imshow("test", color_image_turned);
//		//      cv::waitKey(10);
//	}
//
//	if (display_timing_ == true)
//		ROS_INFO("%d ImageFlip: Time stamp of pointcloud message: %f. Delay: %f.", point_cloud_msg->header.seq, point_cloud_msg->header.stamp.toSec(), ros::Time::now().toSec()-point_cloud_msg->header.stamp.toSec());
}

void ImageFlip::pcConnectCB(const ros::SingleSubscriberPublisher& pub)
{
	pc_sub_counter_++;
	if (pc_sub_counter_ == 1)
	{
		ROS_DEBUG("ImageFlip::pcConnectCB: Connecting point cloud callback.");
		if (pointcloud_data_format_.compare("xyz") == 0)
			point_cloud_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("pointcloud_in", 1, &ImageFlip::pcCallback<pcl::PointXYZ>, this);
		else if (pointcloud_data_format_.compare("xyzrgb") == 0)
			point_cloud_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("pointcloud_in", 1, &ImageFlip::pcCallback<pcl::PointXYZRGB>, this);
		else
		{
			ROS_ERROR("Unknown pointcloud format specified in the paramter file.");
			pc_sub_counter_ = 0;
		}
	}
}

void ImageFlip::pcDisconnectCB(const ros::SingleSubscriberPublisher& pub)
{
	pc_sub_counter_--;
	if (pc_sub_counter_ == 0)
	{
		ROS_DEBUG("ImageFlip::pcDisconnectCB: Disconnecting point cloud callback.");
		point_cloud_sub_.shutdown();
	}
}

}
