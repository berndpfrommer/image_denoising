// -*-c++-*--------------------------------------------------------------------
// Copyright 2021 Bernd Pfrommer <bernd.pfrommer@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "image_denoising/bilateral_filter.h"

#include <opencv2/imgproc/imgproc.hpp>

namespace image_denoising
{
BilateralFilter::BilateralFilter(const ros::NodeHandle & nh) : Filter(nh) {}

BilateralFilter::~BilateralFilter() {}

bool BilateralFilter::initialize()
{
  configServer_.reset(new dynamic_reconfigure::Server<Config>(nh_));
  configServer_->setCallback(
    boost::bind(&BilateralFilter::configure, this, _1, _2));

  return (true);
}

void BilateralFilter::configure(Config & config, int level)
{
  if (level >= 0) {
    if (config_.sigma_color != config.sigma_color) {
      ROS_INFO_STREAM("setting sigma_color to " << config_.sigma_color);
    }
    if (config_.sigma_space != config.sigma_space) {
      ROS_INFO_STREAM("setting sigma_space to " << config_.sigma_space);
    }
  }
  config_ = config;
}

void BilateralFilter::processImage(const StampedImage & si)
{
  stampedImage_.time = si.time;
  cv::bilateralFilter(
    si.image, stampedImage_.image, -1, config_.sigma_color, config_.sigma_space,
    cv::BORDER_DEFAULT);
}

StampedImage BilateralFilter::getNextDenoisedImage() { return (stampedImage_); }

}  // namespace image_denoising
