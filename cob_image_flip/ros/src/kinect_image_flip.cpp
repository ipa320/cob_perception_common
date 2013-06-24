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
 * Author: Richard Bormann, email:richard.bormann@ipa.fhg.de
 * Supervised by: Richard Bormann, email:richard.bormann@ipa.fhg.de
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

#include <cob_image_flip/kinect_image_flip.h>

namespace cob_image_flip
{
CobKinectImageFlip::CobKinectImageFlip(ros::NodeHandle nh)
{
	node_handle_ = nh;

	// set parameters
	std::string robot;
	flip_color_image_ = false;
	flip_pointcloud_ = false;
	pointcloud_data_format_ = "xyz";

	img_sub_counter_ = 0;
	pc_sub_counter_ = 0;

	std::cout << "\n--------------------------\nKinect Image Flip Parameters:\n--------------------------" << std::endl;
	node_handle_.param("flip_color_image", flip_color_image_, false);
	std::cout << "flip_color_image = " << flip_color_image_ << std::endl;
	node_handle_.param("flip_pointcloud", flip_pointcloud_, false);
	std::cout << "flip_pointcloud = " << flip_pointcloud_ << std::endl;
	node_handle_.param<std::string>("pointcloud_data_format", pointcloud_data_format_, "xyz");
	std::cout << "pointcloud_data_format = " << pointcloud_data_format_ << std::endl;
	node_handle_.param("display_warnings", display_warnings_, false);
	std::cout << "display_warnings = " << display_warnings_ << std::endl;
	node_handle_.param("display_timing", display_timing_, false);
	std::cout << "display_timing = " << display_timing_ << std::endl;
	node_handle_.param<std::string>("robot", robot, "cob3-3");
	std::cout << "robot = " << robot << std::endl;

	// determine robot number
	cob3Number_ = 0;
	//ros::get_environment_variable(value, "ROBOT");
	std::stringstream ss;
	ss << robot.substr(robot.find("cob3-")+5);
	ss >> cob3Number_;
	std::cout << "CobKinectImageFlip: Robot number is cob3-" << cob3Number_ << "." << std::endl;

	//sync_pointcloud_ = 0;

	transform_listener_ = 0;

//	std::cout << "a" << std::endl;

	if (flip_color_image_ == true)
	{
		it_ = new image_transport::ImageTransport(node_handle_);

//		std::cout << "a1" << std::endl;


//		std::cout << "a2" << std::endl;

		color_camera_image_sub_.registerCallback(boost::bind(&CobKinectImageFlip::imageCallback, this, _1));

//		std::cout << "a3" << std::endl;

		color_camera_image_pub_ = it_->advertise("colorimage_out", 1, boost::bind(&CobKinectImageFlip::imgConnectCB, this, _1), boost::bind(&CobKinectImageFlip::imgDisconnectCB, this, _1));
	}
	else
	{
		it_ = 0;
	}

//	std::cout << "b" << std::endl;

	// point cloud flip
	if (flip_pointcloud_ == true)
	{
		point_cloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("pointcloud_out", 1,  boost::bind(&CobKinectImageFlip::pcConnectCB, this, _1), boost::bind(&CobKinectImageFlip::pcDisconnectCB, this, _1));
	}

//	std::cout << "c" << std::endl;

	transform_listener_ = new tf::TransformListener(node_handle_);

	std::cout << "CobKinectImageFlip initilized." << std::endl;
}

CobKinectImageFlip::~CobKinectImageFlip()
{
	if (it_ != 0)
		delete it_;
//		if (sync_pointcloud_ != 0)
//			delete sync_pointcloud_;
	if (transform_listener_ != 0)
		delete transform_listener_;
}

unsigned long CobKinectImageFlip::init()
{
	//color_camera_image_sub_.subscribe(*it_, "colorimage", 1);
	//point_cloud_sub_.subscribe(node_handle_, "pointcloud", 1);

	//sync_pointcloud_->connectInput(point_cloud_sub_, color_camera_image_sub_);
	//sync_pointcloud_callback_connection_ = sync_pointcloud_->registerCallback(boost::bind(&CobKinectImageFlipNodelet::inputCallback, this, _1, _2));

	return ipa_Utils::RET_OK;
}

unsigned long CobKinectImageFlip::convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& color_image_msg, cv_bridge::CvImageConstPtr& color_image_ptr, cv::Mat& color_image)
{
	try
	{
		color_image_ptr = cv_bridge::toCvShare(color_image_msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("PeopleDetection: cv_bridge exception: %s", e.what());
		return ipa_Utils::RET_FAILED;
	}
	color_image = color_image_ptr->image;

	return ipa_Utils::RET_OK;
}

template <typename T>
void CobKinectImageFlip::inputCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
//		Timer tim;
//		tim.start();

	// check camera link orientation and decide whether image must be turned around
	bool turnAround = false;
	tf::StampedTransform transform;
	try
	{
		transform_listener_->lookupTransform("/base_link", "/head_cam3d_link", ros::Time(0), transform);
		tfScalar roll, pitch, yaw;
		transform.getBasis().getRPY(roll, pitch, yaw, 1);
		if (cob3Number_ == 2)
			roll *= -1.;
		if (roll > 0.0)
			turnAround = true;
		//      std::cout << "xyz: " << transform.getOrigin().getX() << " " << transform.getOrigin().getY() << " " << transform.getOrigin().getZ() << "\n";
		//      std::cout << "abcw: " << transform.getRotation().getX() << " " << transform.getRotation().getY() << " " << transform.getRotation().getZ() << " " << transform.getRotation().getW() << "\n";
		//      std::cout << "rpy: " << roll << " " << pitch << " " << yaw << "\n";
	} catch (tf::TransformException ex)
	{
		if (display_warnings_ == true)
			ROS_WARN("%s",ex.what());
	}

	if (turnAround == false)
	{
		// image upright
		//sensor_msgs::Image color_image_turned_msg = *color_image_msg;
		//color_image_turned_msg.header.stamp = ros::Time::now();
		//color_camera_image_pub_.publish(color_image_turned_msg);
		//sensor_msgs::PointCloud2::ConstPtr point_cloud_turned_msg = point_cloud_msg;
		point_cloud_pub_.publish(point_cloud_msg);
	}
	else
	{
		// image upside down
		// point cloud
		pcl::PointCloud<T> point_cloud_src;
		pcl::fromROSMsg(*point_cloud_msg, point_cloud_src);

		boost::shared_ptr<pcl::PointCloud<T> > point_cloud_turned(new pcl::PointCloud<T>);
		point_cloud_turned->header = point_cloud_msg->header;
		point_cloud_turned->height = point_cloud_msg->height;
		point_cloud_turned->width = point_cloud_msg->width;
		//point_cloud_turned->sensor_orientation_ = point_cloud_msg->sensor_orientation_;
		//point_cloud_turned->sensor_origin_ = point_cloud_msg->sensor_origin_;
		point_cloud_turned->is_dense = true;	//point_cloud_msg->is_dense;
		point_cloud_turned->resize(point_cloud_src.height*point_cloud_src.width);
		for (int v = (int)point_cloud_src.height - 1; v >= 0; v--)
		{
			for (int u = (int)point_cloud_src.width - 1; u >= 0; u--)
			{
				(*point_cloud_turned)(point_cloud_src.width-1 - u, point_cloud_src.height-1 - v) = point_cloud_src(u, v);
			}
		}

		// publish turned data
		sensor_msgs::PointCloud2::Ptr point_cloud_turned_msg(new sensor_msgs::PointCloud2);
		pcl::toROSMsg(*point_cloud_turned, *point_cloud_turned_msg);
		//point_cloud_turned_msg->header.stamp = ros::Time::now();
		point_cloud_pub_.publish(point_cloud_turned_msg);

		//      cv::namedWindow("test");
		//      cv::imshow("test", color_image_turned);
		//      cv::waitKey(10);
	}

	if (display_timing_ == true)
		ROS_INFO("%d ImageFlip: Time stamp of pointcloud message: %f. Delay: %f.", point_cloud_msg->header.seq, point_cloud_msg->header.stamp.toSec(), ros::Time::now().toSec()-point_cloud_msg->header.stamp.toSec());
//		ROS_INFO("Pointcloud callback in image flip took %f ms.", tim.getElapsedTimeInMilliSec());
}

void CobKinectImageFlip::imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg)
{
//		Timer tim;
//		tim.start();

	// check camera link orientation and decide whether image must be turned around
	bool turnAround = false;
	tf::StampedTransform transform;
	try
	{
		transform_listener_->lookupTransform("/base_link", "/head_cam3d_link", ros::Time(0), transform);
		tfScalar roll, pitch, yaw;
		transform.getBasis().getRPY(roll, pitch, yaw, 1);
		if (cob3Number_ == 2)
			roll *= -1.;
		if (roll > 0.0)
			turnAround = true;
		//      std::cout << "xyz: " << transform.getOrigin().getX() << " " << transform.getOrigin().getY() << " " << transform.getOrigin().getZ() << "\n";
		//      std::cout << "abcw: " << transform.getRotation().getX() << " " << transform.getRotation().getY() << " " << transform.getRotation().getZ() << " " << transform.getRotation().getW() << "\n";
		//      std::cout << "rpy: " << roll << " " << pitch << " " << yaw << "\n";
	} catch (tf::TransformException ex)
	{
		if (display_warnings_ == true)
			ROS_WARN("%s",ex.what());
	}

	// now turn if necessary
	if (turnAround == false)
	{
		// image upright
		sensor_msgs::Image color_image_turned_msg = *color_image_msg;
		color_camera_image_pub_.publish(color_image_turned_msg);
	}
	else
	{
		// image upside down
		cv_bridge::CvImageConstPtr color_image_ptr;
		cv::Mat color_image;
		convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

		// rotate images by 180 degrees
		cv::Mat color_image_turned(color_image.rows, color_image.cols, color_image.type());
		if (color_image.type() != CV_8UC3)
		{
			std::cout << "CobKinectImageFlipNodelet::inputCallback: Error: The image format of the color image is not CV_8UC3.\n";
			return;
		}
		for (int v = 0; v < color_image.rows; v++)
		{
			uchar* src = color_image.ptr(v);
			uchar* dst = color_image_turned.ptr(color_image.rows - v - 1);
			dst += 3 * (color_image.cols - 1);
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
		cv_bridge::CvImage cv_ptr;
		cv_ptr.image = color_image_turned;
		cv_ptr.encoding = "bgr8";
		sensor_msgs::Image::Ptr color_image_turned_msg = cv_ptr.toImageMsg();
		color_image_turned_msg->header = color_image_msg->header;
		color_camera_image_pub_.publish(color_image_turned_msg);
	}

	//ROS_INFO("Image callback in image flip took %f ms.", tim.getElapsedTimeInMilliSec());
}

void CobKinectImageFlip::imgConnectCB(const image_transport::SingleSubscriberPublisher& pub)
  {
    img_sub_counter_++;
    if(img_sub_counter_ == 1)
    {
      ROS_DEBUG("connecting");
      color_camera_image_sub_.subscribe(*it_, "colorimage_in", 1);
    }
  }

void CobKinectImageFlip::imgDisconnectCB(const image_transport::SingleSubscriberPublisher& pub)
  {
    img_sub_counter_--;
    if(img_sub_counter_ == 0)
    {
      ROS_DEBUG("disconnecting");
      color_camera_image_sub_.unsubscribe();
    }
  }

void CobKinectImageFlip::pcConnectCB(const ros::SingleSubscriberPublisher& pub)
  {
    pc_sub_counter_++;
    if(pc_sub_counter_ == 1)
    {
      ROS_DEBUG("connecting");
      if (pointcloud_data_format_.compare("xyz") == 0)
	point_cloud_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("pointcloud_in", 1, &CobKinectImageFlip::inputCallback<pcl::PointXYZ>, this);
      else if (pointcloud_data_format_.compare("xyzrgb") == 0)
	point_cloud_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("pointcloud_in", 1, &CobKinectImageFlip::inputCallback<pcl::PointXYZRGB>, this);
      else
      {
	ROS_ERROR("Unknown pointcloud format specified in the paramter file.");
	pc_sub_counter_ = 0;
      }
    }
  }

void CobKinectImageFlip::pcDisconnectCB(const ros::SingleSubscriberPublisher& pub)
  {
    pc_sub_counter_--;
    if(pc_sub_counter_ == 0)
    {
      ROS_DEBUG("disconnecting");
      point_cloud_sub_.shutdown();
    }
  }

}
