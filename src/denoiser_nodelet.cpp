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

#include <nodelet/nodelet.h>
#include <ros/ros.h>

#include <memory>

#include "image_denoising/denoiser.h"

namespace image_denoising
{
class DenoiserNodelet : public nodelet::Nodelet
{
public:
  void onInit() override
  {
    nh_ = getPrivateNodeHandle();
    const std::string filter = nh_.param<std::string>("filter", "nl_means");
    denoiser_ = std::make_shared<Denoiser>(nh_);
    if (!denoiser_->initialize(filter)) {
      ROS_ERROR("cannot initialize denoiser");
      throw std::runtime_error("cannot initialize denoiser!");
    }
  }

private:
  // ------ variables --------
  std::shared_ptr<image_denoising::Denoiser> denoiser_;
  ros::NodeHandle nh_;
};
}  // namespace image_denoising

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(image_denoising::DenoiserNodelet, nodelet::Nodelet)
