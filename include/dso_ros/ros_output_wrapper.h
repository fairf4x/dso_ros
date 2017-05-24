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

#ifndef DSO_ROS_ROS_IO_WRAPPER
#define DSO_ROS_ROS_IO_WRAPPER

#include <FullSystem/HessianBlocks.h>
#include <IOWrapper/Output3DWrapper.h>
#include <util/FrameShell.h>
#include <util/MinimalImage.h>
#include <util/settings.h>

#include <Eigen/Core>

#include <boost/thread.hpp>

#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/duration.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/mat.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace dso_ros
{
class ROSOutputWrapper : public dso::IOWrap::Output3DWrapper
{
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;

  struct DSOCameraParams {
    float fxi, fyi, cxi, cyi;
    DSOCameraParams(dso::CalibHessian* HCalib)
    {
      float fx = HCalib->fxl();
      float fy = HCalib->fyl();
      float cx = HCalib->cxl();
      float cy = HCalib->cyl();
      fxi = 1 / fx;
      fyi = 1 / fy;
      cxi = -cx / fx;
      cyi = -cy / fy;
    }
  };

public:
  ROSOutputWrapper(ros::NodeHandle& n);

  virtual ~ROSOutputWrapper();

  virtual void publishKeyframes(std::vector<dso::FrameHessian*>& frames,
                                bool final, dso::CalibHessian* HCalib) override;

  virtual void publishCamPose(dso::FrameShell* frame,
                              dso::CalibHessian* HCalib, 
			      Eigen::Matrix<Sophus::SE3Group<double>::Scalar, 3, 4>* transformation) override;

  virtual void pushLiveFrame(dso::FrameHessian* image) override;

  virtual void pushDepthImage(dso::MinimalImageB3* image) override;

  virtual bool needPushDepthImage() override;

  virtual void pushDepthImageFloat(dso::MinimalImageF* image,
                                   dso::FrameHessian* KF) override;
  virtual void reset() override
  {
    reset_ = true;
    ROS_ERROR_STREAM("[DSO_NODE]: RESET Current position: "
                     << pose_.getOrigin().getX() << ", "
                     << pose_.getOrigin().getY() << ", "
                     << pose_.getOrigin().getZ());
  }

  void pushTimestamp(const ros::Time& time)
  {
    timestamp_ = time;
  }

private:
  /* camera frame id */
  std::string dso_frame_id_;
  std::string camera_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;
  /* dso odometry_msg */
  ros::Publisher dso_odom_pub_;
  ros::Publisher dso_depht_image_pub_;
  ros::Publisher pcl_pub_;
  tf::TransformListener tf_list_;
  tf::TransformBroadcaster tf_broadcast_;

  tf::Transform last_pose_;
  tf::Transform pose_;

  size_t seq_image_;
  bool reset_;
  ros::Time timestamp_;
  int last_id_;

  std::vector<Point> DSOtoPcl(const dso::PointHessian* pt,
                              const DSOCameraParams& params) const;
};
}
#endif  // DSO_ROS_ROS_IO_WRAPPER
