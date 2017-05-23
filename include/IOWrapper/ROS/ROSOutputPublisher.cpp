/**
* This file is part of DSO.
*
* Copyright 2016 Technical University of Munich and Intel.
* Developed by Jakob Engel <engelj at in dot tum dot de>,
* for more information see <http://vision.in.tum.de/dso>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* DSO is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* DSO is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with DSO. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ROSOutputPublisher.h"
#include "Eigen/Core"
#include "IOWrapper/Output3DWrapper.h"
#include "boost/thread.hpp"
#include "util/MinimalImage.h"

#include "FullSystem/HessianBlocks.h"
#include "util/FrameShell.h"

#include <ros/console.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>

namespace dso
{
namespace IOWrap
{
ROSOutputPublisher::ROSOutputPublisher(ros::NodeHandle dso_node)
{
	std::ofstream fileKey;
	fileKey.open ("/home/harasim/catkin_ws/rosviewer.log");
	fileKey << std::setprecision(15);

			fileKey << "\n";

	fileKey.close();

  ROS_INFO("ROSOutputPublisher created\n");
  if (!dso_node.hasParam("dso_frame_id")) {
    ROS_INFO("No param named world_frame found!");
  }
  if (!dso_node.hasParam("camera_frame_id")) {
    ROS_INFO("No param named camera_frame found!");
  }
  dso_node.param<std::string>("dso_frame_id", dso_frame_id, "dso_camera");
  dso_node.param<std::string>("camera_frame_id", camera_frame_id, "camera");
  dso_node.param<std::string>("odom_frame_id", odom_frame_id, "odom");
  dso_node.param<std::string>("base_frame_id", base_frame_id, "base_link");

  ROS_INFO_STREAM("dso_frame_id = " << dso_frame_id << "\n");
  ROS_INFO_STREAM("camera_frame_id = " << camera_frame_id << "\n");
  ROS_INFO_STREAM("odom_frame_id = " << odom_frame_id << "\n");
  ROS_INFO_STREAM("base_frame_id = " << base_frame_id << "\n");

  dso_odom_pub =
      dso_node.advertise<nav_msgs::Odometry>("dso_odom_topic", 5, false);

lastCamPose.position.x = 0;
lastCamPose.position.y = 0;
lastCamPose.position.z = 0;
}

ROSOutputPublisher::~ROSOutputPublisher()
{
  ROS_INFO("ROSOutputPublisher destroyed\n");
}

void ROSOutputPublisher::publishGraph(
    const std::map<long, Eigen::Vector2i>& connectivity)
{
  /*printf("OUT: got graph with %d edges\n", (int)connectivity.size());

  int maxWrite = 5;

  for (const std::pair<long, Eigen::Vector2i>& p : connectivity) {
    int idHost = p.first >> 32;
    int idTarget = p.first & 0xFFFFFFFF;
    printf("OUT: Example Edge %d -> %d has %d active and %d marg residuals\n",
           idHost, idTarget, p.second[0], p.second[1]);
    maxWrite--;
    if (maxWrite == 0)
      break;
  }*/
}

void ROSOutputPublisher::publishKeyframes(std::vector<FrameHessian*>& frames,
                                          bool final, CalibHessian* HCalib)
{
  /*for (FrameHessian* f : frames) {
    printf("OUT: KF %d (%s) (id %d, tme %f): %d active, %d marginalized, %d "
           "immature points. CameraToWorld:\n",
           f->frameID, final ? "final" : "non-final", f->shell->incoming_id,
           f->shell->timestamp, (int)f->pointHessians.size(),
           (int)f->pointHessiansMarginalized.size(),
           (int)f->immaturePoints.size());
    std::cout << f->shell->camToWorld.matrix3x4() << "\n";

    int maxWrite = 5;
    for (PointHessian* p : f->pointHessians) {
      printf("OUT: Example Point x=%.1f, y=%.1f, idepth=%f, idepth std.dev. "
             "%f, %d inlier-residuals\n",
             p->u, p->v, p->idepth_scaled, sqrt(1.0f / p->idepth_hessian),
             p->numGoodResiduals);
      maxWrite--;
      if (maxWrite == 0)
        break;
    }
  }*/
}

void ROSOutputPublisher::publishCamPose(FrameShell* frame, CalibHessian* HCalib, Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 4>* transformation)
{
  tf::StampedTransform tf_base_cam;

  /* This function broadcasts tf transformation:
world->cam based on frame->camToWorld.matrix3x4()

  frame->camToWorld.matrix3x4() returns:

        m00 m01 m02 m03
        m10 m11 m12 m13
        m20 m21 m22 m23

        last column is translation vector
        3x3 matrix with diagonal m00 m11 and m22 is a rotation matrix
        */

/*	std::ofstream fileKey;
	fileKey.open ("/home/harasim/catkin_ws/rosviewer.log", std::ios_base::app);
	fileKey << std::setprecision(15);

			fileKey << frame->id <<
			" " << frame->camToWorld.translation()[0] << 
			" " << frame->camToWorld.translation()[1] <<
			" " << frame->camToWorld.translation()[2] << 
			" " << sizeof frame->camToWorld.translation() << 
			" " << sizeof frame->camToWorld.translation()[0] << 
			" " << std::addressof(frame->camToWorld.translation()) << " ";

	long size = sizeof frame->camToWorld.translation();
	char* buffer = new char[size];

	reinterpret_cast<char*>(std::addressof(frame->camToWorld.translation()));

	for ( unsigned int i = 0; i < size; i++)
	fileKey << buffer[i];

	fileKey << "\n";

	delete[] buffer;
	fileKey.close();*/

  Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 4> m = *transformation;
//double x = frame->camToWorld.translation().x();
//double y = frame->camToWorld.translation().y();
//double z = frame->camToWorld.translation().z();
  /* camera position */
  double camX = m(0, 3);
  double camY = m(1, 3);
  double camZ = m(2, 3);

ROS_INFO("Id = %d, Trans %f, %f, %f",frame->id, camX, camY,camZ);
  //ROS_INFO("Cam to world \n A=%f %f %f %f\n  %f %f %f %f\n  %f %f %f %f",
	//	m(0,0), m(0,1), m(0,2), m(0,3), 
	//	m(1,0), m(1,1), m(1,2), m(1,3),
	//	m(2,0), m(2,1), m(2,2), m(2,3));

  Eigen::Quaterniond qe(m.block<3,3>(0,0));

  /* broadcast map -> cam_pose transformation */
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(camX, camY, camZ));
  tf::Quaternion q = tf::Quaternion(qe.x(), qe.y(), qe.z(), qe.w());
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                                        odom_frame_id, dso_frame_id));

  nav_msgs::Odometry odom;
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = odom_frame_id;

  geometry_msgs::Pose pose;
//  intentional swap of coordinates
  pose.position.x = camX;
  pose.position.y = camY;
  pose.position.z = camZ;
  pose.orientation.x = qe.x();
  pose.orientation.y = qe.y();
  pose.orientation.z = qe.z();
  pose.orientation.w = qe.w();

  odom.pose.pose = pose;
  odom.twist.twist.linear.x = pose.position.x - lastCamPose.position.x;
  odom.twist.twist.linear.y = pose.position.y - lastCamPose.position.y;
  odom.twist.twist.linear.z = pose.position.z - lastCamPose.position.z;
  //tf::poseTFToMsg(transform.origin(), odom.pose.pose);

  if(!std::isnan(camX)&&!std::isnan(camY)&&!std::isnan(camZ)
&& !std::isnan(qe.x()) && !std::isnan(qe.y())&& !std::isnan(qe.z())&&!std::isnan(qe.w())
&&!std::isinf(camX)&&!std::isinf(camY)&&!std::isinf(camZ)
&& !std::isinf(qe.x()) && !std::isinf(qe.y())&& !std::isinf(qe.z())&&!std::isinf(qe.w()))
{
  dso_odom_pub.publish(odom);
}

  lastCamPose = pose;
}

void ROSOutputPublisher::printResult(std::string file) {}

void ROSOutputPublisher::pushLiveFrame(FrameHessian* image)
{
  // can be used to get the raw image / intensity pyramid.
}

void ROSOutputPublisher::pushDepthImage(MinimalImageB3* image)
{
  // can be used to get the raw image with depth overlay.
}
bool ROSOutputPublisher::needPushDepthImage()
{
  return false;
}

void ROSOutputPublisher::pushDepthImageFloat(MinimalImageF* image,
                                             FrameHessian* KF)
{
  /*printf("OUT: Predicted depth for KF %d (id %d, time %f, internal frame-ID "
         "%d). CameraToWorld:\n",
         KF->frameID, KF->shell->incoming_id, KF->shell->timestamp,
         KF->shell->id);
  std::cout << KF->shell->camToWorld.matrix3x4() << "\n";

  int maxWrite = 5;
  for (int y = 0; y < image->h; y++) {
    for (int x = 0; x < image->w; x++) {
      if (image->at(x, y) <= 0)
        continue;

      printf("OUT: Example Idepth at pixel (%d,%d): %f.\n", x, y,
             image->at(x, y));
      maxWrite--;
      if (maxWrite == 0)
        break;
    }
    if (maxWrite == 0)
      break;
  }*/
}
}
}
