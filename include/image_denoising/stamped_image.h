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

#ifndef IMAGE_DENOISING_STAMPED_IMAGE_H_
#define IMAGE_DENOISING_STAMPED_IMAGE_H_

#include <ros/ros.h>

#include <opencv2/core/core.hpp>

namespace image_denoising
{
struct StampedImage
{
  StampedImage() {}
  explicit StampedImage(const ros::Time & t) : time(t) {}
  StampedImage(const ros::Time & t, const cv::Mat & img) : time(t), image(img)
  {
  }
  ros::Time time;
  cv::Mat image;
};
}  // namespace image_denoising
#endif
