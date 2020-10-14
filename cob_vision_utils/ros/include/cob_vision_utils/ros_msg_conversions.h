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


#ifndef COB_VISION_UTILS_ROS_MSG_CONVERSIONS_H__
#define COB_VISION_UTILS_ROS_MSG_CONVERSIONS_H__

#include <cob_object_detection_msgs/Detection.h>
#include <visualization_msgs/Marker.h>

#include <Eigen/Geometry> // for transformations

namespace cob_perception_common
{

  // Eigen to ROS conversions
  inline void EigenToROSMsg(const Eigen::Vector3f& pt_eigen, geometry_msgs::Point& pt_msg)
  { pt_msg.x=pt_eigen[0]; pt_msg.y=pt_eigen[1]; pt_msg.z=pt_eigen[2]; }

  inline void EigenToROSMsg(const Eigen::Quaternion<float>& q_eigen, geometry_msgs::Quaternion& q_msg)
  { q_msg.x=q_eigen.x(); q_msg.y=q_eigen.y(); q_msg.z=q_eigen.z(); q_msg.w=q_eigen.w(); }

  inline void EigenToROSMsg(const Eigen::Vector3f& pt, const Eigen::Quaternion<float>& rot, geometry_msgs::Pose& pose)
  { EigenToROSMsg(pt, pose.position); EigenToROSMsg(rot, pose.orientation); }


  // ROS to Eigen conversions
  inline void ROSMsgToEigen(const geometry_msgs::Point& pt_msg, Eigen::Vector3f& pt_eigen)
  { pt_eigen[0]=pt_msg.x; pt_eigen[1]=pt_msg.y; pt_eigen[2]=pt_msg.z; }

  inline void ROSMsgToEigen(const geometry_msgs::Quaternion& q_msg, Eigen::Quaternion<float>& q_eigen)
  { q_eigen.x()=q_msg.x; q_eigen.y()=q_msg.y; q_eigen.z()=q_msg.z; q_eigen.w()=q_msg.w; }

  inline void ROSMsgToEigen(const geometry_msgs::Pose& pose, Eigen::Vector3f& pt, Eigen::Quaternion<float>& rot)
  { ROSMsgToEigen(pose.position, pt); ROSMsgToEigen(pose.orientation, rot); }



  // Marker properties type, lifetime, header and id should be set manually;
  void boundingBoxToMarker(const cob_object_detection_msgs::Detection& bb, visualization_msgs::Marker& marker)
  {
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    geometry_msgs::Point pt;

    /*************************
     * Lines between point indices of 0-1, 2-3, 4-5, ...
     *
     * bottom of bounding box:
     *
     *   1,2,10   0,7,8
     *     +-------+
     *     |       |
     *     +-------+
     *   3,4,12   5,6,14
     *
     * top of bounding box:
     *
     * 11,17,18   9,16,23
     *     +-------+
     *     |       |
     *     +-------+
     * 13,19,20   15,21,22
     *
     *************************/

    marker.points.resize(24);
    pt.x = bb.bounding_box_lwh.x;
    pt.y = bb.bounding_box_lwh.y;
    pt.z = 0;

    marker.points[0]=marker.points[7]=marker.points[8] = pt;

    pt.x = -bb.bounding_box_lwh.x;
    marker.points[1]=marker.points[2]=marker.points[10] = pt;

    pt.y = -bb.bounding_box_lwh.y;
    marker.points[3]=marker.points[4]=marker.points[12] = pt;

    pt.x = bb.bounding_box_lwh.x;
    marker.points[5]=marker.points[6]=marker.points[14] = pt;


    pt.y = bb.bounding_box_lwh.y;
    pt.z = bb.bounding_box_lwh.z;

    marker.points[9]=marker.points[16]=marker.points[23] = pt;

    pt.x = -bb.bounding_box_lwh.x;
    marker.points[11]=marker.points[17]=marker.points[18] = pt;

    pt.y = -bb.bounding_box_lwh.y;
    marker.points[13]=marker.points[19]=marker.points[20] = pt;

    pt.x = bb.bounding_box_lwh.x;
    marker.points[15]=marker.points[21]=marker.points[22] = pt;

    marker.pose = bb.pose.pose;

    marker.scale.x = 0.01;
    marker.color.r = 0;
    marker.color.g = 1;
    marker.color.b = 0;
    marker.color.a = 1.0;
  }
}

#endif // COB_VISION_UTILS_ROS_MSG_CONVERSIONS_H__
