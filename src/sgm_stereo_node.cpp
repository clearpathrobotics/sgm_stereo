/*
    Copyright (C) 2017  Vaibhav Mehta

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

#include <pluginlib/class_list_macros.h>
#include "sgm_stereo/sgm_stereo_node.h"


PLUGINLIB_DECLARE_CLASS(sgm_stereo, StereoSGMNode, sgm_stereo::StereoSGMNode, nodelet::Nodelet);


namespace sgm_stereo
{

StereoSGMNode::StereoSGMNode()
{}


void StereoSGMNode::onInit()
{
  left_image_sub_.subscribe(nh_, "left/image_rect", 1);
  right_image_sub_.subscribe(nh_, "right/image_rect", 1);
  left_info_sub_.subscribe(nh_, "left/camera_info", 1);
  right_info_sub_.subscribe(nh_, "right/camera_info", 1);

  const auto l_image_msg = ros::topic::waitForMessage<sensor_msgs::Image>(left_image_sub_.getTopic(), nh_);
  const auto r_image_msg = ros::topic::waitForMessage<sensor_msgs::Image>(right_image_sub_.getTopic(), nh_);

  if (l_image_msg->width != r_image_msg->width || l_image_msg->height != r_image_msg->height)
  {
    ROS_FATAL_STREAM("Left " << l_image_msg->width << "x" << l_image_msg->height <<
        " and Right " << r_image_msg->width << "x" << r_image_msg->height << " images do NOT have the same size!");
    ros::shutdown();
    return;
  }

  ros::NodeHandle nh_priv("~");

  // SGM Params
  int disparity_total = 128;
  double disparity_factor = 256;
  int sobel_cap_value = 15;
  int census_window_radius = 2;
  double census_weight_factor = 1.0/6.0;
  int aggregration_window_radius = 2;
  int smoothness_penalty_small = 100;
  int smoothness_penalty_large = 1600;
  int consistency_threshold = 1;
  int max_speckle_size = 100;
  bool enforce_left_right_consistency = true;

  nh_priv.param<int>("disparity_total", disparity_total, disparity_total);
  nh_priv.param<double>("disparity_factor", disparity_factor, disparity_factor);
  nh_priv.param<int>("sobel_cap_value", sobel_cap_value, sobel_cap_value);
  nh_priv.param<int>("census_window_radius", census_window_radius, census_window_radius);
  nh_priv.param<double>("census_weight_factor", census_weight_factor, census_weight_factor);
  nh_priv.param<int>("aggregration_window_radius", aggregration_window_radius, aggregration_window_radius);
  nh_priv.param<int>("smoothness_penalty_small", smoothness_penalty_small, smoothness_penalty_small);
  nh_priv.param<int>("smoothness_penalty_large", smoothness_penalty_large, smoothness_penalty_large);
  nh_priv.param<int>("consistency_threshold", consistency_threshold, consistency_threshold);
  nh_priv.param<int>("max_speckle_size", max_speckle_size, max_speckle_size);
  nh_priv.param<bool>("enforce_left_right_consistency", enforce_left_right_consistency, enforce_left_right_consistency);

  // SGM Node Params
  nh_priv.param<int>("textureless_thresh", textureless_thresh_, 300);
  nh_priv.param<bool>("filter_textureless_regions", filter_textureless_regions_, false);
  nh_priv.param<bool>("display_debugging_images", display_debugging_images_, false);
  nh_priv.param<bool>("rotate_images", rotate_images_, false);

  sgm_.setParams(disparity_total, disparity_factor, sobel_cap_value,
                 census_window_radius, census_weight_factor,
                 aggregration_window_radius, smoothness_penalty_small,
                 smoothness_penalty_large, consistency_threshold,
                 max_speckle_size);

  sgm_.setEnforceLeftRightConsistency(enforce_left_right_consistency);

  sgm_.initialize(l_image_msg->width, l_image_msg->height);

  sync_.reset(new Synchronizer(ImageSyncPolicy(5), left_image_sub_, right_image_sub_, left_info_sub_, right_info_sub_)),
  sync_->registerCallback(boost::bind(&StereoSGMNode::callback, this, _1, _2, _3, _4));

  pub_disparity_ = nh_.advertise<stereo_msgs::DisparityImage>("disparity", 1);
}

void StereoSGMNode::convertOpenCV2ROSImage( const cv::Mat& opencv_img,
                                            const std::string& image_encoding,
                                            sensor_msgs::Image& ros_img)
{
  cv_bridge::CvImage cv_bridge_img;
  cv_bridge_img.encoding = image_encoding;
  cv_bridge_img.image = opencv_img;

  cv_bridge_img.toImageMsg(ros_img);
}

void StereoSGMNode::computeSGMStereoDisparity(const sensor_msgs::ImageConstPtr& l_image_msg,
                                              const sensor_msgs::ImageConstPtr& r_image_msg,
                                              const image_geometry::StereoCameraModel& model,
                                              stereo_msgs::DisparityImage& disp_msg)
{
  // Get OpenCV Image Mats (grayscale)
  const auto l_image = cv_bridge::toCvShare(l_image_msg, sensor_msgs::image_encodings::MONO8)->image;
  const auto r_image = cv_bridge::toCvShare(r_image_msg, sensor_msgs::image_encodings::MONO8)->image;

  cv::Mat left_gray_img = l_image.clone();
  cv::Mat right_gray_img = r_image.clone();

  // ------------------- Compute Mask for Textureless Left regions --------------
  cv::Mat left_grad;
  extractSobelVerticalEdges(left_gray_img, "Left Gray ", left_grad);
  cv::Mat left_textureless_mask  = cv::Mat::zeros(left_gray_img.size(), CV_8U);
  extractTextureLessRegion(left_grad, left_textureless_mask);

  if (display_debugging_images_)
  {
    cv::Mat left_gray_img_textured  = cv::Mat::zeros(left_gray_img.size(), CV_8U);
    left_gray_img.copyTo(left_gray_img_textured, left_textureless_mask);

    if (rotate_images_)
    {
      cv::flip(left_gray_img_textured, left_gray_img_textured, -1);
    }

    cv::imshow("left_gray_img_textured", left_gray_img_textured);
    cv::waitKey(10);
  }


  // --------------------------------------------------------------
  float disparityImage[l_image.cols * l_image.rows];

  sgm_.compute(l_image, r_image, disparityImage);
  cv::Mat disparity_temp(l_image.rows, l_image.cols, CV_32F, disparityImage);

  // Apply Textured Mask
  cv::Mat disparity = cv::Mat::zeros(disparity_temp.size(), CV_32F);

  if (filter_textureless_regions_)
  {
    disparity_temp.copyTo(disparity, left_textureless_mask);
  }

  disp_msg.header            = l_image_msg->header;

  disp_msg.f                 = model.left().fx();
  disp_msg.T                 = model.baseline();
  disp_msg.min_disparity     = 0.0;
  disp_msg.max_disparity     = 127.0;
  disp_msg.delta_d           = 0.0625;

  disp_msg.image.header      = l_image_msg->header;
  disp_msg.image.height      = l_image.rows;
  disp_msg.image.width       = l_image.cols;

  if (filter_textureless_regions_)
  {
    convertOpenCV2ROSImage(disparity, sensor_msgs::image_encodings::TYPE_32FC1, disp_msg.image);
  }
  else
  {
    convertOpenCV2ROSImage(disparity_temp, sensor_msgs::image_encodings::TYPE_32FC1, disp_msg.image);
  }

}

bool StereoSGMNode::isValidPoint(const cv::Vec3f& pt)
{
  // Check both for disparities explicitly marked as invalid (where OpenCV maps pt.z to MISSING_Z)
  // and zero disparities (point mapped to infinity).
  return pt[2] != image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt[2]);
}

void StereoSGMNode::extractTextureLessRegion(const cv::Mat& edge_img, cv::Mat& textureless_mask)
{
  for (int x = 0; x < edge_img.cols; x++)
  {
    for (int y = 0; y < edge_img.rows; y++)
    {
      if (sumNeighborEdgeGradients(edge_img, x, y) >= textureless_thresh_)
      {
        textureless_mask.at<uchar>(y, x) = 255;
      }
    }
  }

  if (display_debugging_images_)
  {
    if (rotate_images_)
    {
      cv::Mat textureless_mask_rot = textureless_mask.clone();
      cv::flip(textureless_mask_rot, textureless_mask_rot, -1);
      cv::imshow("textureless_mask", textureless_mask_rot);
    }
    else
    {
      cv::imshow("textureless_mask", textureless_mask);
    }
    cv::waitKey(10);
  }
}

int StereoSGMNode::sumNeighborEdgeGradients(const cv::Mat& edge_img, int x, int y)
{
  int x_inds[] = {-2, -1, 0, 1, 2,        -2, -1, 0, 1, 2,       -2, -1, 0, 1, 2,     -2, -1, 0, 1, 2,      -2, -1, 0, 1, 2};
  int y_inds[] = { 2,  2, 2, 2, 2,         1,  1, 1, 1, 1,        0,  0, 0, 0, 0,     -1, -1, -1, -1, -1,   -2, -2, -2, -2, -2};

  // TODO: make neighborhood size configurable
  int n = 25;

  int sum_region_gray = 0;

  for (int i = 0; i < n; i++)
  {
    int nx = x + x_inds[i];
    int ny = y + y_inds[i];

    // Check if pixels out of bounds
    if (nx < 0 || nx >= edge_img.cols || ny < 0 || ny >= edge_img.rows)
    {
      continue;
    }

    uchar gray = edge_img.at<uchar>(ny, nx);

    sum_region_gray += (int)gray;
  }

  return sum_region_gray;
}


void StereoSGMNode::extractSobelVerticalEdges(const cv::Mat& gray_img, std::string title, cv::Mat& grad)
{
  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;

  int ddepth = CV_16S;
  int kernel_size = 3;
  int scale = 1;
  int delta = 0;

  // Gradient X
  cv::Sobel(gray_img, grad_x, ddepth, 1, 0, kernel_size, scale, delta, cv::BORDER_DEFAULT );
  cv::convertScaleAbs(grad_x, abs_grad_x);

  // Gradient Y
  cv::Sobel(gray_img, grad_y, ddepth, 0, 1, kernel_size, scale, delta, cv::BORDER_DEFAULT );
  cv::convertScaleAbs(grad_y, abs_grad_y);

  // Total Gradient (approximate)
  cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad);

  if (display_debugging_images_)
  {
    if (rotate_images_)
    {
      cv::Mat grad_rot = grad.clone();
      cv::flip(grad_rot, grad_rot, -1);
      cv::imshow(title + "Sobel Total Gradient", grad_rot);
    }
    else
    {
      cv::imshow(title + "Sobel Total Gradient", grad);
    }
    cv::waitKey(10);
  }
}

void StereoSGMNode::callback(const sensor_msgs::ImageConstPtr& left_ros_img,
                              const sensor_msgs::ImageConstPtr& right_ros_img,
                              const sensor_msgs::CameraInfoConstPtr& left_info,
                              const sensor_msgs::CameraInfoConstPtr& right_info)
{
  // update the camera info. Do it once as it's only needed once -
  model_.fromCameraInfo(left_info, right_info);

  stereo_msgs::DisparityImage disp_msg;

  // Time SGM Stereo Disparity Computation
  ros::Time begin = ros::Time::now();
  computeSGMStereoDisparity(left_ros_img, right_ros_img, model_, disp_msg);
  ros::Time end = ros::Time::now();
  ros::Duration duration = end - begin;
  ROS_INFO("Duration of [computeSGMStereoDisparity]: %.2f sec", duration.toSec());

  pub_disparity_.publish(disp_msg);
}

}  // namespace sgm_stereo
