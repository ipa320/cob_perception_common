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
PLUGINLIB_DECLARE_CLASS(cob_image_flip, ImageFlipNodelet, cob_image_flip::ImageFlipNodelet, nodelet::Nodelet)
