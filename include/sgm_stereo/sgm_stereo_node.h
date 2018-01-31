/*
    Copyright (C) 2018  Teyvonia Thomas

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SGM_STEREO_SGM_STEREO_NODE_H
#define SGM_STEREO_SGM_STEREO_NODE_H

// C++ Includes
#include <string>
#include <vector>
#include <cstdint>
#include <iostream>
#include <cstring>
#include <stack>
#include <ctime>

// ROS Includes
#include <ros/ros.h>
#include <ros/topic.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// OpenCV Includes
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "nodelet/nodelet.h"
#include "sgm_stereo/SGMStereo.h"


namespace sgm_stereo
{

class StereoSGMNode : public nodelet::Nodelet
{
public:
  StereoSGMNode();

private:
  virtual void onInit();

  void callback(const sensor_msgs::ImageConstPtr& left,
                const sensor_msgs::ImageConstPtr& right,
                const sensor_msgs::CameraInfoConstPtr& left_info,
                const sensor_msgs::CameraInfoConstPtr& right_info);

  void convertOpenCV2ROSImage(const cv::Mat& opencv_img,
                              const std::string& image_encoding,
                              sensor_msgs::Image& ros_img);

  void computeSGMStereoDisparity(const sensor_msgs::ImageConstPtr& l_image_msg,
                                  const sensor_msgs::ImageConstPtr& r_image_msg,
                                  const image_geometry::StereoCameraModel& model,
                                  stereo_msgs::DisparityImage& disp_msg);

  bool isValidPoint(const cv::Vec3f& pt);

  // Methods for Extracting/Filtering Textureless Regions for Disparity Maps
  void extractSobelVerticalEdges(const cv::Mat& gray_img, std::string title, cv::Mat& grad);
  int sumNeighborEdgeGradients(const cv::Mat& edge_img, int x, int y);
  void extractTextureLessRegion(const cv::Mat& edge_img, cv::Mat& textureless_mask);

protected:
  ros::NodeHandle nh_;
  std::string image_encoding_;

  message_filters::Subscriber<sensor_msgs::Image> left_image_sub_;
  message_filters::Subscriber<sensor_msgs::Image> right_image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> left_info_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> right_info_sub_;

  // Sync Policy for Images
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::Image, sensor_msgs::Image,
    sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ImageSyncPolicy;

  typedef message_filters::Synchronizer<ImageSyncPolicy> Synchronizer;
  boost::shared_ptr<Synchronizer> sync_;

  image_geometry::StereoCameraModel model_;

  SGMStereo sgm_;
  ros::Publisher pub_disparity_;
  bool display_debugging_images_;
  bool rotate_images_;

  // Extract TextureLess Regions param
  int textureless_thresh_;
  bool filter_textureless_regions_;
};

}  // namespace sgm_stereo

#endif  // SGM_STEREO_SGM_STEREO_NODE_H
