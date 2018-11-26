/*
 * Copyright 2018 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
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

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>

class FOVNode
{
public:
  /**
   * \brief Constructor.
   */
  FOVNode ()
  {
      pub_marker_ = n_.advertise<visualization_msgs::Marker>("cam_fov",10);
      sub_cam_ = n_.subscribe("camera_info", 10, &FOVNode::camCallback, this);
  }

  /** Empty Destructor */
  ~FOVNode ()
  {
    /// void
  }

private:
  ros::NodeHandle n_; //!< ROS node handle.

  ros::Publisher pub_marker_;
  ros::Subscriber sub_cam_;
  void camCallback(const sensor_msgs::CameraInfo::ConstPtr& msg);
};

void FOVNode::camCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
        bool show_marker;
        double min_distance, max_distance;
        ros::param::get("~show_marker", show_marker);
        ros::param::get("~min_distance", min_distance);
        ros::param::get("~max_distance", max_distance);

        if(show_marker==true)
        {
            sensor_msgs::CameraInfo camerainfo = *msg;	// get camera infos
            image_geometry::PinholeCameraModel pinmodel;	// get pinhole model from header
            pinmodel.fromCameraInfo(camerainfo);

            visualization_msgs::Marker cam_poly;	// create marker
            cam_poly.header = camerainfo.header;
            cam_poly.action = visualization_msgs::Marker::ADD;
            cam_poly.id = 0;
            cam_poly.ns = "fov";
            cam_poly.type = visualization_msgs::Marker::LINE_STRIP;	// ..as line strip
            cam_poly.scale.x = 0.01;
            cam_poly.color.g = 1.0; // set colors
            cam_poly.color.b = 1.0;
            cam_poly.color.a = 0.5; // set transparency

            // calc 3D Points out of boundaries of image. Result are four points  (rect at 1m distance)
            cv::Point3d P_topleft = pinmodel.projectPixelTo3dRay(cv::Point(0, 0));
            cv::Point3d P_downright = pinmodel.projectPixelTo3dRay(cv::Point(camerainfo.width, camerainfo.height));
            cv::Point3d P_topright = pinmodel.projectPixelTo3dRay(cv::Point(camerainfo.width, 0));
            cv::Point3d P_downleft = pinmodel.projectPixelTo3dRay(cv::Point(0, camerainfo.height));


            // project rect into desired distances (min_distance and max_distance)

            // vertices front rect
            geometry_msgs::Point p1, p2, p3, p4, p5, p6, p7, p8;
            p1.x = P_topleft.x * min_distance;
            p1.y = P_topleft.y * min_distance;
            p1.z = P_topleft.z * min_distance;

            p2.x = P_topright.x * min_distance;
            p2.y = P_topright.y * min_distance;
            p2.z = P_topright.z * min_distance;

            p3.x = P_downright.x * min_distance;
            p3.y = P_downright.y * min_distance;
            p3.z = P_downright.z * min_distance;

            p4.x = P_downleft.x * min_distance;
            p4.y = P_downleft.y * min_distance;
            p4.z = P_downleft.z * min_distance;

            // vertices rear rect
            p5.x = P_topleft.x * max_distance;
            p5.y = P_topleft.y * max_distance;
            p5.z = P_topleft.z * max_distance;

            p6.x = P_topright.x * max_distance;
            p6.y = P_topright.y * max_distance;
            p6.z = P_topright.z * max_distance;

            p7.x= P_downright.x * max_distance;
            p7.y= P_downright.y * max_distance;
            p7.z= P_downright.z * max_distance;

            p8.x= P_downleft.x * max_distance;
            p8.y= P_downleft.y * max_distance;
            p8.z= P_downleft.z * max_distance;

            // push back points to get line polynom
            cam_poly.points.push_back(p1);
            cam_poly.points.push_back(p2);
            cam_poly.points.push_back(p3);
            cam_poly.points.push_back(p4);
            cam_poly.points.push_back(p1);
            cam_poly.points.push_back(p5);
            cam_poly.points.push_back(p6);
            cam_poly.points.push_back(p7);
            cam_poly.points.push_back(p8);
            cam_poly.points.push_back(p5);
            cam_poly.points.push_back(p8);
            cam_poly.points.push_back(p4);
            cam_poly.points.push_back(p3);
            cam_poly.points.push_back(p7);
            cam_poly.points.push_back(p6);
            cam_poly.points.push_back(p2);

            pub_marker_.publish(cam_poly);


        }
}

int main(int argc, char *argv[])
  {
    ros::init(argc, argv, "visualize_cam_fov");
    FOVNode fov;
    ros::spin();
    return 0;
  }
