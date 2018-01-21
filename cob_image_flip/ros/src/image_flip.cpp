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

#include <cob_image_flip/image_flip.h>
#include <geometry_msgs/TransformStamped.h>

namespace cob_image_flip
{
ImageFlip::ImageFlip(ros::NodeHandle nh)
: 	node_handle_(nh), img_sub_counter_(0), pc_sub_counter_(0), disparity_sub_counter_(0), transform_listener_(nh), it_(0), last_rotation_angle_(0), last_rotation_factor_(0)
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
	node_handle_.param("flip_disparity_image", flip_disparity_image_, false);
	ROS_DEBUG_STREAM("flip_disparity_image = " << flip_disparity_image_);
	node_handle_.param("display_warnings", display_warnings_, false);
	ROS_DEBUG_STREAM("display_warnings = " << display_warnings_);

	if (flip_color_image_ == true)
	{
		it_ = new image_transport::ImageTransport(node_handle_);
		color_camera_image_sub_.registerCallback(boost::bind(&ImageFlip::imageCallback, this, _1));
		color_camera_image_pub_ = it_->advertise("colorimage_out", 1, boost::bind(&ImageFlip::imgConnectCB, this, _1), boost::bind(&ImageFlip::imgDisconnectCB, this, _1));
		color_camera_image_2d_transform_pub_ = node_handle_.advertise<cob_perception_msgs::Float64ArrayStamped>("colorimage_inplane_transform", 1,false);
	}

	// point cloud flip
	if (flip_pointcloud_ == true)
	{
		point_cloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("pointcloud_out", 1,  boost::bind(&ImageFlip::pcConnectCB, this, _1), boost::bind(&ImageFlip::pcDisconnectCB, this, _1));
		point_cloud_2d_transform_pub_ = node_handle_.advertise<cob_perception_msgs::Float64ArrayStamped>("pointcloud_inplane_transform", 1,false);
	}

	if (flip_disparity_image_ == true)
	{
		disparity_image_pub_ = node_handle_.advertise<stereo_msgs::DisparityImage>("disparityimage_out", 1,  boost::bind(&ImageFlip::disparityConnectCB, this, _1), boost::bind(&ImageFlip::disparityDisconnectCB, this, _1));
		disparity_image_2d_transform_pub_ = node_handle_.advertise<cob_perception_msgs::Float64ArrayStamped>("disparityimage_inplane_transform", 1,false);
	}

	ROS_DEBUG_STREAM("ImageFlip initialized.");
}

ImageFlip::~ImageFlip()
{
	if (it_ != 0)
		delete it_;
}


double ImageFlip::determineRotationAngle(const std::string& camera_frame_id, const ros::Time& time)
{
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
			tf::Stamped<tf::Vector3> x_axis_camera(tf::Vector3(1, 0, 0), time /*ros::Time(0)*/, camera_frame_id), x_axis_ref;
			tf::Stamped<tf::Vector3> y_axis_camera(tf::Vector3(0, 1, 0), x_axis_camera.stamp_, camera_frame_id), y_axis_ref;
			transform_listener_.waitForTransform(reference_frame_, camera_frame_id, x_axis_camera.stamp_, ros::Duration(0.2));
			transform_listener_.transformVector(reference_frame_, x_axis_camera, x_axis_ref);
			transform_listener_.transformVector(reference_frame_, y_axis_camera, y_axis_ref);
			// compute the rotation angle so that the x-axis of the rotated image has 0 z-coordinate in the reference coordinate system (i.e. x-axis of rotated image is then parallel to the ground)
			// the rotation is around the z-axis of the camera system
			// 1. Compute the intersection between
			//    E_1: x-y-plane in reference system coordinates: z=0
			//    E_2: x-y-plane of camera system (but without translation, just the rotational part): [x,y,z] = [0,0,0] + r*[x_axis_ref.x,x_axis_ref.y,x_axis_ref.z] + s*[y_axis_ref.x,y_axis_ref.y,y_axis_ref.z]
			//    --> r = -s*y_axis_ref.z/x_axis_ref.z
			//    --> line equation: x = s*[y_axis_ref.x-x_axis_ref.x*y_axis_ref.z/x_axis_ref.z,y_axis_ref.y-x_axis_ref.y*y_axis_ref.z/x_axis_ref.z,0]
			// 2. Compute the target camera x-axis, which is parallel to the ground. The vector for x is given by the line equation, the direction of the target y-axis decides about the direction (+ or -).
			// 3. Compute the rotation between camera x-axis and target camera x-axis (i.e. between x_axis_ref and x_axis_target)
			if (x_axis_ref.z()!=0)		// do not compute a rotation if the camera's x-axis is already correctly aligned
			{
				// 1. line intersection
				const double a = y_axis_ref.z()/x_axis_ref.z();
				tf::Vector3 x_axis_target(y_axis_ref.x()-x_axis_ref.x()*a,y_axis_ref.y()-x_axis_ref.y()*a, 0);		// this is the line of intersection
				x_axis_target.normalize();
				// 2. resolve direction ambiguity
				tf::Vector3 z_axis_target = x_axis_ref.cross(y_axis_ref);	// remark: z_axis_ref == z_axis_target
				tf::Vector3 y_axis_target = z_axis_target.cross(x_axis_target);
				y_axis_target.normalize();

				//std::cout << "\n\nx_axis_ref=" << x_axis_ref.x() << ", " << x_axis_ref.y() << ", " << x_axis_ref.z()
				//		<< "\ny_axis_ref=" << y_axis_ref.x() << ", " << y_axis_ref.y() << ", " << y_axis_ref.z()
				//		<< "\nx_axis_target=" << x_axis_target.x() << ", " << x_axis_target.y() << ", " << x_axis_target.z()
				//		<< "\ny_axis_target=" << y_axis_target.x() << ", " << y_axis_target.y() << ", " << y_axis_target.z() << "\n" << std::endl;

				// compute a factor than rotates the image in a way that the new y-axis in the rotated image directs against the z-direction of the reference system (i.e. points downwards)
				int factor = (y_axis_target.z()<0. ? 1 : -1);
				if (factor != last_rotation_factor_ && fabs(y_axis_target.z()) < 0.01)		// this hysteresis stops continuous flipping near 0
					factor = last_rotation_factor_;
				last_rotation_factor_ = factor;
				x_axis_target *= factor;

				//std::cout << "x_axis_target factored=" << x_axis_target.x() << ", " << x_axis_target.y() << ", " << x_axis_target.z() << "\n" << std::endl;

				// 3. compute angle
				tf::Vector3 rot_axis_x = x_axis_ref.cross(x_axis_target);
				double rot_sin = ((rot_axis_x.dot(z_axis_target)) >= 0 ? 1 : -1) * rot_axis_x.length();		// sign of sin() depends on alignment of cross-product rotation axis with z-axis
				double rot_cos = x_axis_ref.dot(x_axis_target);
				rotation_angle = -180./CV_PI * atan2(rot_sin, rot_cos);

				//std::cout << "rot_sin=" << rot_sin << "\trot_cos=" << rot_cos << "\trotation_angle=" << rotation_angle << "\n=========================================\n" << std::endl;
			}
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
				ROS_DEBUG("%s",ex.what());
			rotation_angle = last_rotation_angle_;
		}
	}
	else
	{
		if (display_warnings_)
			ROS_WARN("ImageFlip::imageCallback: Unsupported rotation mode.");
	}

	return rotation_angle;
}


bool ImageFlip::convertImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
{
	try
	{
		image_ptr = cv_bridge::toCvShare(image_msg, image_msg->encoding);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("ImageFlip::convertColorImageMessageToMat: cv_bridge exception: %s", e.what());
		return false;
	}
	image = image_ptr->image;

	return true;
}


void ImageFlip::imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg)
{
	// read image
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;
	if (convertImageMessageToMat(color_image_msg, color_image_ptr, color_image) == false)
		return;
	cv::Mat color_image_turned;
	cv::Mat rot_mat = cv::Mat::zeros(2,3,CV_64FC1);

	// determine rotation angle
	double rotation_angle = determineRotationAngle(color_image_msg->header.frame_id, color_image_msg->header.stamp);

	// rotate
	// fast hard coded rotations
	if (rotation_angle==0. || rotation_angle==360. || rotation_angle==-360.)
	{
		color_image_turned = color_image;
		rot_mat.at<double>(0,0) = 1.;
		rot_mat.at<double>(1,1) = 1.;
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
		rot_mat.at<double>(0,1) = -1.;
		rot_mat.at<double>(0,2) = color_image.rows;
		rot_mat.at<double>(1,0) = 1.;
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
		rot_mat.at<double>(0,1) = 1.;
		rot_mat.at<double>(1,0) = -1.;
		rot_mat.at<double>(1,2) = color_image.cols;
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
		rot_mat.at<double>(0,0) = -1.;
		rot_mat.at<double>(0,2) = color_image.cols;
		rot_mat.at<double>(1,1) = -1.;
		rot_mat.at<double>(1,2) = color_image.rows;
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
		rot_mat = cv::getRotationMatrix2D(center, -rotation_angle, 1.0);
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
	cv_ptr.encoding = color_image_msg->encoding;
	sensor_msgs::Image::Ptr color_image_turned_msg = cv_ptr.toImageMsg();
	color_image_turned_msg->header = color_image_msg->header;
	color_camera_image_pub_.publish(color_image_turned_msg);

	// publish rotation matrix for backward transform to non-turned coordinates using a homogeneous image coordinate representation
	// the original, non-turned coordinates are of interest if the camera calibration shall be used (the calibration does not apply to coordinates of the turned image)
	cv::Mat rot33 = cv::Mat::eye(3,3,CV_64FC1);
	for (int r=0; r<2; ++r)
		for (int c=0; c<3; ++c)
			rot33.at<double>(r,c) = rot_mat.at<double>(r,c);
	cv::Mat rot33_inv = rot33.inv();
	cob_perception_msgs::Float64ArrayStamped rot33_inv_msg;
	rot33_inv_msg.header = color_image_msg->header;
	rot33_inv_msg.data.resize(9);
	for (int r=0; r<3; ++r)
		for (int c=0; c<3; ++c)
			rot33_inv_msg.data[r*3+c] = rot33_inv.at<double>(r,c);
	color_camera_image_2d_transform_pub_.publish(rot33_inv_msg);
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


void ImageFlip::pcCallback(const sensor_msgs::PointCloud2::ConstPtr& point_cloud_msg)
{
	// determine rotation angle
	double rotation_angle = determineRotationAngle(point_cloud_msg->header.frame_id, point_cloud_msg->header.stamp);
	cv::Mat rot_mat = cv::Mat::zeros(2,3,CV_64FC1);

	// prepare turned point cloud
	const int element_size = point_cloud_msg->point_step;	// length of point in bytes
	const int row_size = point_cloud_msg->row_step;		// length of row in bytes
	sensor_msgs::PointCloud2::Ptr point_cloud_out_msg = boost::shared_ptr<sensor_msgs::PointCloud2>(new sensor_msgs::PointCloud2);
	point_cloud_out_msg->fields = point_cloud_msg->fields;
	point_cloud_out_msg->header = point_cloud_msg->header;
	point_cloud_out_msg->height = point_cloud_msg->height;
	point_cloud_out_msg->width = point_cloud_msg->width;
	point_cloud_out_msg->point_step = point_cloud_msg->point_step;
	point_cloud_out_msg->row_step = point_cloud_msg->row_step;
	point_cloud_out_msg->is_bigendian = point_cloud_msg->is_bigendian;
	point_cloud_out_msg->is_dense = point_cloud_msg->is_dense;
	point_cloud_out_msg->data.resize(point_cloud_out_msg->height * point_cloud_out_msg->width * element_size);

	// rotate
	// fast hard coded rotations
	if (rotation_angle==0. || rotation_angle==360. || rotation_angle==-360.)
	{
		memcpy(&(point_cloud_out_msg->data[0]), &(point_cloud_msg->data[0]), point_cloud_msg->height*point_cloud_msg->width*element_size);
		rot_mat.at<double>(0,0) = 1.;
		rot_mat.at<double>(1,1) = 1.;
	}
	else if (rotation_angle==90. || rotation_angle==-270.)
	{
		// rotate images by 90 degrees
		point_cloud_out_msg->height = point_cloud_msg->width;
		point_cloud_out_msg->width = point_cloud_msg->height;
		const int row_size_out = point_cloud_msg->height*element_size;		// length of row in bytes
		point_cloud_out_msg->row_step = row_size_out;
		for (int v = 0; v < point_cloud_out_msg->height; v++)
			for (int u = 0; u < point_cloud_out_msg->width; u++)
				memcpy(&(point_cloud_out_msg->data[v*row_size_out+u*element_size]), &(point_cloud_msg->data[(point_cloud_msg->height-1-u)*row_size+v*element_size]), element_size);
		rot_mat.at<double>(0,1) = -1.;
		rot_mat.at<double>(0,2) = point_cloud_msg->height;
		rot_mat.at<double>(1,0) = 1.;
	}
	else if (rotation_angle==270. || rotation_angle==-90.)
	{
		// rotate images by 270 degrees
		point_cloud_out_msg->height = point_cloud_msg->width;
		point_cloud_out_msg->width = point_cloud_msg->height;
		const int row_size_out = point_cloud_msg->height*element_size;		// length of row in bytes
		point_cloud_out_msg->row_step = row_size_out;
		for (int v = 0; v < point_cloud_out_msg->height; v++)
			for (int u = 0; u < point_cloud_out_msg->width; u++)
				memcpy(&(point_cloud_out_msg->data[v*row_size_out+u*element_size]), &(point_cloud_msg->data[u*row_size+(point_cloud_msg->width-1-v)*element_size]), element_size);
		rot_mat.at<double>(0,1) = 1.;
		rot_mat.at<double>(1,0) = -1.;
		rot_mat.at<double>(1,2) = point_cloud_msg->width;
	}
	else if (rotation_angle==180 || rotation_angle==-180)
	{
		// rotate images by 180 degrees
		for (int v = 0; v < point_cloud_out_msg->height; v++)
			for (int u = 0; u < point_cloud_out_msg->width; u++)
				memcpy(&(point_cloud_out_msg->data[v*row_size+u*element_size]), &(point_cloud_msg->data[(point_cloud_msg->height-1-v)*row_size+(point_cloud_msg->width-1-u)*element_size]), element_size);
		rot_mat.at<double>(0,0) = -1.;
		rot_mat.at<double>(0,2) = point_cloud_msg->width;
		rot_mat.at<double>(1,1) = -1.;
		rot_mat.at<double>(1,2) = point_cloud_msg->height;
	}
	else
	{
		// arbitrary rotation
		int row_size_out = row_size;
		// automatically decide for landscape or portrait orientation of resulting image
		bool switch_aspect_ratio = !(fabs(sin(rotation_angle*CV_PI/180.)) < 0.707106781);
		if (switch_aspect_ratio==true)
		{
			point_cloud_out_msg->height = point_cloud_msg->width;
			point_cloud_out_msg->width = point_cloud_msg->height;
			row_size_out = point_cloud_msg->height*element_size;		// length of row in bytes
			point_cloud_out_msg->row_step = row_size_out;
		}

		// compute transform
		cv::Point center = cv::Point(point_cloud_msg->width/2, point_cloud_msg->height/2);
		rot_mat = cv::getRotationMatrix2D(center, -rotation_angle, 1.0);
		if (switch_aspect_ratio==true)
		{
			rot_mat.at<double>(0,2) += 0.5*((double)point_cloud_out_msg->width - (double)point_cloud_msg->width);
			rot_mat.at<double>(1,2) += 0.5*((double)point_cloud_out_msg->height - (double)point_cloud_msg->height);
		}
		cv::Mat rot_mat_inv = rot_mat.clone();
		rot_mat_inv.at<double>(0,1) = rot_mat.at<double>(1,0);
		rot_mat_inv.at<double>(1,0) = rot_mat.at<double>(0,1);
		rot_mat_inv.at<double>(0,2) = -rot_mat.at<double>(0,2)*rot_mat.at<double>(0,0) - rot_mat.at<double>(1,2)*rot_mat.at<double>(1,0);
		rot_mat_inv.at<double>(1,2) = -rot_mat.at<double>(1,2)*rot_mat.at<double>(0,0) + rot_mat.at<double>(0,2)*rot_mat.at<double>(1,0);

		// zero element
		std::vector<uchar> zero_element(element_size, 0);
		for (size_t i=0; i<point_cloud_msg->fields.size(); ++i)
		{
			if (point_cloud_msg->fields[i].name.compare("x") == 0 || point_cloud_msg->fields[i].name.compare("y") == 0 || point_cloud_msg->fields[i].name.compare("z") == 0)
			{
				float* val = (float*)&(zero_element[point_cloud_msg->fields[i].offset]);
				*val = std::numeric_limits<float>::quiet_NaN();
			}
			if (point_cloud_msg->fields[i].name.compare("rgb") == 0)
			{
				float* val = (float*)&(zero_element[point_cloud_msg->fields[i].offset]);
				int vali = 0;
				*val = *(float*)(&vali);
			}
		}

		// warp (nearest neighbor mode, no interpolation)
		for (int v = 0; v < point_cloud_out_msg->height; v++)
		{
			for (int u = 0; u < point_cloud_out_msg->width; u++)
			{
				int src_u = rot_mat_inv.at<double>(0,0)*u + rot_mat_inv.at<double>(0,1)*v + rot_mat_inv.at<double>(0,2);
				int src_v = rot_mat_inv.at<double>(1,0)*u + rot_mat_inv.at<double>(1,1)*v + rot_mat_inv.at<double>(1,2);
				if (src_u < 0 || src_u > point_cloud_msg->width-1 || src_v < 0 || src_v > point_cloud_msg->height-1)
					memcpy(&(point_cloud_out_msg->data[v*row_size_out+u*element_size]), &(zero_element[0]), element_size);
				else
					memcpy(&(point_cloud_out_msg->data[v*row_size_out+u*element_size]), &(point_cloud_msg->data[src_v*row_size+src_u*element_size]), element_size);
			}
		}
	}

	// publish turned point cloud
	point_cloud_pub_.publish(point_cloud_out_msg);

	// publish rotation matrix for backward transform to non-turned coordinates using a homogeneous image coordinate representation
	// the original, non-turned coordinates are of interest if the camera calibration shall be used (the calibration does not apply to coordinates of the turned image)
	cv::Mat rot33 = cv::Mat::eye(3,3,CV_64FC1);
	for (int r=0; r<2; ++r)
		for (int c=0; c<3; ++c)
			rot33.at<double>(r,c) = rot_mat.at<double>(r,c);
	cv::Mat rot33_inv = rot33.inv();
	cob_perception_msgs::Float64ArrayStamped rot33_inv_msg;
	rot33_inv_msg.header = point_cloud_msg->header;
	rot33_inv_msg.data.resize(9);
	for (int r=0; r<3; ++r)
		for (int c=0; c<3; ++c)
			rot33_inv_msg.data[r*3+c] = rot33_inv.at<double>(r,c);
	point_cloud_2d_transform_pub_.publish(rot33_inv_msg);


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
		point_cloud_sub_ = node_handle_.subscribe<sensor_msgs::PointCloud2>("pointcloud_in", 1, &ImageFlip::pcCallback, this);
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

void void_delete_image_msg(const sensor_msgs::Image*)
{
	return;
}

void ImageFlip::disparityCallback(const stereo_msgs::DisparityImage::ConstPtr& disparity_image_msg)
{
	// read image
	cv_bridge::CvImageConstPtr disparity_image_ptr;
	cv::Mat disparity_image;
	sensor_msgs::ImageConstPtr disparity_image_constptr = boost::shared_ptr<const sensor_msgs::Image>(&disparity_image_msg->image, void_delete_image_msg);
	if (convertImageMessageToMat(disparity_image_constptr, disparity_image_ptr, disparity_image) == false)
		return;
	cv::Mat disparity_image_turned;
	cv::Mat rot_mat = cv::Mat::zeros(2,3,CV_64FC1);

	// determine rotation angle
	double rotation_angle = determineRotationAngle(disparity_image_msg->header.frame_id, disparity_image_msg->header.stamp);

	// rotate
	// fast hard coded rotations
	if (rotation_angle==0. || rotation_angle==360. || rotation_angle==-360.)
	{
		disparity_image_turned = disparity_image;
		rot_mat.at<double>(0,0) = 1.;
		rot_mat.at<double>(1,1) = 1.;
	}
	else if (rotation_angle==90. || rotation_angle==-270.)
	{
		// rotate images by 90 degrees
		disparity_image_turned.create(disparity_image.cols, disparity_image.rows, disparity_image.type());
		if (disparity_image.type() != CV_32FC1)
		{
			ROS_ERROR("ImageFlip::imageCallback: Error: The image format of the disparity image is not CV_32FC1.\n");
			return;
		}
		for (int v = 0; v < disparity_image_turned.rows; v++)
			for (int u = 0; u < disparity_image_turned.cols; u++)
				disparity_image_turned.at<float>(v,u) = disparity_image.at<float>(disparity_image.rows-1-u,v);
		rot_mat.at<double>(0,1) = -1.;
		rot_mat.at<double>(0,2) = disparity_image.rows;
		rot_mat.at<double>(1,0) = 1.;
	}
	else if (rotation_angle==270. || rotation_angle==-90.)
	{
		// rotate images by 270 degrees
		disparity_image_turned.create(disparity_image.cols, disparity_image.rows, disparity_image.type());
		if (disparity_image.type() != CV_32FC1)
		{
			std::cout << "ImageFlip::imageCallback: Error: The image format of the color image is not CV_32FC1.\n";
			return;
		}
		for (int v = 0; v < disparity_image_turned.rows; v++)
			for (int u = 0; u < disparity_image_turned.cols; u++)
				disparity_image_turned.at<float>(v,u) = disparity_image.at<float>(u,disparity_image.cols-1-v);
		rot_mat.at<double>(0,1) = 1.;
		rot_mat.at<double>(1,0) = -1.;
		rot_mat.at<double>(1,2) = disparity_image.cols;
	}
	else if (rotation_angle==180 || rotation_angle==-180)
	{
		// rotate images by 180 degrees
		disparity_image_turned.create(disparity_image.rows, disparity_image.cols, disparity_image.type());
		if (disparity_image.type() != CV_32FC1)
		{
			std::cout << "ImageFlip::imageCallback: Error: The image format of the color image is not CV_32FC1.\n";
			return;
		}
		for (int v = 0; v < disparity_image.rows; v++)
		{
			float* src = (float*)disparity_image.ptr(v);
			float* dst = (float*)disparity_image_turned.ptr(disparity_image.rows - v - 1) + (disparity_image.cols - 1);
			for (int u = 0; u < disparity_image.cols; u++)
			{
				*dst = *src;
				src++;
				dst--;
			}
		}
		rot_mat.at<double>(0,0) = -1.;
		rot_mat.at<double>(0,2) = disparity_image.cols;
		rot_mat.at<double>(1,1) = -1.;
		rot_mat.at<double>(1,2) = disparity_image.rows;
	}
	else
	{
		// arbitrary rotation
		bool switch_aspect_ratio = !(fabs(sin(rotation_angle*CV_PI/180.)) < 0.707106781);
		if (switch_aspect_ratio==false)
			disparity_image_turned.create(disparity_image.rows, disparity_image.cols, disparity_image.type());		// automatically decide for landscape or portrait orientation of resulting image
		else
			disparity_image_turned.create(disparity_image.cols, disparity_image.rows, disparity_image.type());
		if (disparity_image.type() != CV_32FC1)
		{
			ROS_ERROR("ImageFlip::imageCallback: Error: The image format of the color image is not CV_32FC1.\n");
			return;
		}

		cv::Point center = cv::Point(disparity_image.cols/2, disparity_image.rows/2);
		rot_mat = cv::getRotationMatrix2D(center, -rotation_angle, 1.0);
		if (switch_aspect_ratio==true)
		{
			rot_mat.at<double>(0,2) += 0.5*(disparity_image_turned.cols-disparity_image.cols);
			rot_mat.at<double>(1,2) += 0.5*(disparity_image_turned.rows-disparity_image.rows);
		}
		cv::warpAffine(disparity_image, disparity_image_turned, rot_mat, disparity_image_turned.size());
	}

	// publish turned image
	cv_bridge::CvImage cv_ptr;
	cv_ptr.image = disparity_image_turned;
	cv_ptr.encoding = disparity_image_msg->image.encoding;
	stereo_msgs::DisparityImage::Ptr disparity_image_turned_msg(new stereo_msgs::DisparityImage);
	sensor_msgs::ImagePtr disparity_image_turned_msg_image = cv_ptr.toImageMsg();
	disparity_image_turned_msg_image->header = disparity_image_msg->image.header;
	disparity_image_turned_msg->image = *disparity_image_turned_msg_image;
	disparity_image_turned_msg->header = disparity_image_msg->header;
	disparity_image_pub_.publish(disparity_image_turned_msg);

	// publish rotation matrix for backward transform to non-turned coordinates using a homogeneous image coordinate representation
	// the original, non-turned coordinates are of interest if the camera calibration shall be used (the calibration does not apply to coordinates of the turned image)
	cv::Mat rot33 = cv::Mat::eye(3,3,CV_64FC1);
	for (int r=0; r<2; ++r)
		for (int c=0; c<3; ++c)
			rot33.at<double>(r,c) = rot_mat.at<double>(r,c);
	cv::Mat rot33_inv = rot33.inv();
	cob_perception_msgs::Float64ArrayStamped rot33_inv_msg;
	rot33_inv_msg.header = disparity_image_msg->header;
	rot33_inv_msg.data.resize(9);
	for (int r=0; r<3; ++r)
		for (int c=0; c<3; ++c)
			rot33_inv_msg.data[r*3+c] = rot33_inv.at<double>(r,c);
	disparity_image_2d_transform_pub_.publish(rot33_inv_msg);
}


void ImageFlip::disparityConnectCB(const ros::SingleSubscriberPublisher& pub)
{
	disparity_sub_counter_++;
	if (disparity_sub_counter_ == 1)
	{
		ROS_DEBUG("ImageFlip::disparityConnectCB: Connecting disparity callback.");
		disparity_image_sub_ = node_handle_.subscribe<stereo_msgs::DisparityImage>("disparityimage_in", 1, &ImageFlip::disparityCallback, this);
	}
}

void ImageFlip::disparityDisconnectCB(const ros::SingleSubscriberPublisher& pub)
{
	disparity_sub_counter_--;
	if (disparity_sub_counter_ == 0)
	{
		ROS_DEBUG("ImageFlip::disparityDisconnectCB: Disconnecting disparity callback.");
		disparity_image_sub_.shutdown();
	}
}


}
