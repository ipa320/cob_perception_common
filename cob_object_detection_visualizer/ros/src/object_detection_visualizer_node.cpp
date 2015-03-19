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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "cob_object_detection_msgs/DetectionArray.h"


class ObjectDetectionVisualizer
{
public:
	ObjectDetectionVisualizer(ros::NodeHandle& nh)
	: node_handle_(nh), prev_marker_array_size_(0)
	{
		detection_array_sub_ = node_handle_.subscribe("detection_array_topic", 1, &ObjectDetectionVisualizer::objectDetectionArrayCallback, this);
		marker_array_publisher_ = node_handle_.advertise<visualization_msgs::MarkerArray>("object_detection_marker_array", 0);
	}

	~ObjectDetectionVisualizer()
	{

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

	ros::NodeHandle node_handle_;
	ros::Subscriber detection_array_sub_;
	ros::Publisher marker_array_publisher_;

	unsigned int prev_marker_array_size_;
	visualization_msgs::MarkerArray marker_array_msg_;
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
