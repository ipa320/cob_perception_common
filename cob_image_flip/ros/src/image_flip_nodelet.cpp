/*
 * Copyright 2017 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
 

//##################
//#### includes ####

// ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>

// image flip code
#include <cob_image_flip/image_flip.h>

// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

namespace cob_image_flip
{
class ImageFlipNodelet : public nodelet::Nodelet
{
protected:
	ros::NodeHandle node_handle_;
	ImageFlip* image_flip_;

public:

	ImageFlipNodelet()
	{
		image_flip_ = 0;
	};

	~ImageFlipNodelet()
	{
		if (image_flip_ != 0)
			delete image_flip_;
	};

	virtual void onInit()
	{
		// Create a handle for this node, initialize node
		node_handle_ = getPrivateNodeHandle();

		// Create ImageFlip class instance
		image_flip_ = new ImageFlip(node_handle_);
	}
};
}

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(cob_image_flip::ImageFlipNodelet, nodelet::Nodelet)
