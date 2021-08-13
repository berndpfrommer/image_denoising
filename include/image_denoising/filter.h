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

#ifndef IMAGE_DENOISING_FILTER_H_
#define IMAGE_DENOISING_FILTER_H_

#include <ros/ros.h>

#include <memory>
#include <opencv2/core/core.hpp>

#include "image_denoising/stamped_image.h"

namespace image_denoising
{
class Filter
{
public:
  Filter(const ros::NodeHandle & nh) : nh_(nh) {}
  virtual ~Filter(){};
  virtual bool initialize() = 0;
  virtual void processImage(const StampedImage & img) = 0;
  // may return a null image if no denoised image is available yet
  virtual StampedImage getNextDenoisedImage() = 0;

  // factory method
  static std::shared_ptr<Filter> make(
    const ros::NodeHandle & nh, const std::string & name);

protected:
  // ------ variables ---------
  ros::NodeHandle nh_;
};
}  // namespace image_denoising

#endif
