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

#include "image_denoising/filter.h"

#include <ros/ros.h>

#include "image_denoising/bilateral_filter.h"
#include "image_denoising/nl_means_filter.h"

namespace image_denoising
{
std::shared_ptr<Filter> Filter::make(
  const ros::NodeHandle & nh, const std::string & name)
{
  if (name == "nl_means") {
    return (std::make_shared<NLMeansFilter>(nh));
  } else if (name == "bilateral") {
    return (std::make_shared<BilateralFilter>(nh));
  } else {
    ROS_ERROR_STREAM("invalid filter name: " << name);
    throw std::runtime_error("invalid filter name!");
  }
}
}  // namespace image_denoising
