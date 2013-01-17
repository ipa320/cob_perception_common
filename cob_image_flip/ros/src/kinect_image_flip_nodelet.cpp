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

// ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// topics
//#include <message_filters/subscriber.h>
//#include <message_filters/synchronizer.h>
//#include <message_filters/sync_policies/approximate_time.h>
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

// timer
#include <cob_image_flip/timer.h>

// image flip code
#include <cob_image_flip/kinect_image_flip.h>

// System
#include <iostream>

#include <cob_vision_utils/GlobalDefines.h>


// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

namespace cob_image_flip
{
class CobKinectImageFlipNodelet : public nodelet::Nodelet
{
protected:
	ros::NodeHandle node_handle_;
	CobKinectImageFlip* cob_kinect_image_flip_;

public:

	CobKinectImageFlipNodelet()
	{
		cob_kinect_image_flip_ = 0;
	};

	~CobKinectImageFlipNodelet()
	{
		if (cob_kinect_image_flip_ != 0)
			delete cob_kinect_image_flip_;
	};

	virtual void onInit()
	{
		// Create a handle for this node, initialize node
		node_handle_ = getPrivateNodeHandle();

		// Create CobKinectImageFlip class instance
		cob_kinect_image_flip_ = new CobKinectImageFlip(node_handle_);
	}
};
}

// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(cob_image_flip, CobKinectImageFlipNodelet, cob_image_flip::CobKinectImageFlipNodelet, nodelet::Nodelet)
