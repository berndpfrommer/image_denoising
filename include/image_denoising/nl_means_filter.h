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

#ifndef IMAGE_DENOISING_NL_MEANS_FILTER_H_
#define IMAGE_DENOISING_NL_MEANS_FILTER_H_

#include <dynamic_reconfigure/server.h>

#include <deque>
#include <memory>
#include <opencv2/core/core.hpp>

#include "image_denoising/NLMeansDynConfig.h"
#include "image_denoising/filter.h"

namespace image_denoising
{
class NLMeansFilter : public Filter
{
public:
  using Config = NLMeansDynConfig;

  NLMeansFilter(const ros::NodeHandle & nh);
  ~NLMeansFilter();
  // --- inherited from Filter ---
  bool initialize() override;
  void processImage(const StampedImage & img) override;
  StampedImage getNextDenoisedImage() override;

private:
  StampedImage denoiseSingleImage(const StampedImage & si) const;
  StampedImage denoiseCenterImage() const;
  void configure(Config & config, int level);
  // ------ variables ---------
  std::shared_ptr<dynamic_reconfigure::Server<Config>> configServer_;
  std::deque<StampedImage> imageQueue_;
  float h_{3.0};
  int templateWindowSize_{7};
  int searchWindowSize_{21};
  size_t maxQueueSize_{3};
};
}  // namespace image_denoising

#endif
