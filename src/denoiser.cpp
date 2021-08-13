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

#include "image_denoising/denoiser.h"

#include <cv_bridge/cv_bridge.h>

namespace image_denoising
{
Denoiser::Denoiser(const ros::NodeHandle & nh) : nh_(nh) {}

Denoiser::~Denoiser() {}

bool Denoiser::initialize(const std::string & filter)
{
  filter_ = Filter::make(nh_, filter);
  if (!filter_->initialize()) {
    ROS_ERROR_STREAM("failed to initialize filter: " << filter);
    return (false);
  }
  imageTransport_ = std::make_shared<image_transport::ImageTransport>(nh_);
  imagePub_ = imageTransport_->advertise(
    "denoised_image", nh_.param<int>("ros_send_queue_size", 200));
  imageSub_ = imageTransport_->subscribe(
    "image", nh_.param<int>("ros_recv_queue_size", 200),
    boost::bind(&Denoiser::imageCallback, this, _1));
  ROS_INFO_STREAM("denoiser initialized with filter: " << filter);
  return (true);
}

void Denoiser::imageCallback(const sensor_msgs::ImageConstPtr & image)
{
  // decode image
  cv_bridge::CvImageConstPtr cp = cv_bridge::toCvShare(image, "mono8");
  filter_->processImage(StampedImage(image->header.stamp, cp->image));
  StampedImage denoised = filter_->getNextDenoisedImage();
  if (denoised.image.rows > 0 && denoised.image.cols > 0) {
    imagePub_.publish(
      cv_bridge::CvImage(image->header, "mono8", denoised.image).toImageMsg());
  } else {
    ROS_INFO_STREAM("insufficient frames yet to publish image!");
  }
}
}  // namespace image_denoising
