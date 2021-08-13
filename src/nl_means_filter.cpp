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

#include "image_denoising/nl_means_filter.h"

#include <opencv2/photo/photo.hpp>

namespace image_denoising
{
static int make_odd(int i) { return (i % 2 == 0 ? i + 1 : i); }

NLMeansFilter::NLMeansFilter(const ros::NodeHandle & nh) : Filter(nh) {}

NLMeansFilter::~NLMeansFilter() {}

bool NLMeansFilter::initialize()
{
  configServer_.reset(new dynamic_reconfigure::Server<Config>(nh_));
  configServer_->setCallback(
    boost::bind(&NLMeansFilter::configure, this, _1, _2));

  return (true);
}

void NLMeansFilter::configure(Config & config, int level)
{
  (void)level;
  maxQueueSize_ = make_odd(config.num_frames);
  config.num_frames = maxQueueSize_;
  templateWindowSize_ = make_odd(config.template_window_size);
  config.template_window_size = templateWindowSize_;
  searchWindowSize_ = make_odd(config.search_window_size);
  config.search_window_size = searchWindowSize_;
  h_ = config.h;
}

void NLMeansFilter::processImage(const StampedImage & si)
{
  imageQueue_.push_front(si);
  while (imageQueue_.size() > maxQueueSize_) {
    imageQueue_.pop_back();
  }
}

StampedImage NLMeansFilter::getNextDenoisedImage()
{
  if (imageQueue_.size() <= maxQueueSize_ / 2) {
    // just do single image denoising while the queue
    // is being built up
    return (denoiseSingleImage(*(imageQueue_.rbegin())));
  } else if (imageQueue_.size() == maxQueueSize_) {
    // got enough in the queue to denoise the center image
    return (denoiseCenterImage());
  }
  // cannot denoise yet because queue is not full yet, return null pointer
  return (StampedImage());
}

StampedImage NLMeansFilter::denoiseSingleImage(const StampedImage & si) const
{
  StampedImage denoised(si.time);
  cv::fastNlMeansDenoising(
    si.image, denoised.image, h_, templateWindowSize_, searchWindowSize_);
  return (denoised);
}

StampedImage NLMeansFilter::denoiseCenterImage() const
{
  const int centerIdx = imageQueue_.size() / 2;
  const auto & centerImage = imageQueue_[centerIdx];
  StampedImage denoised(centerImage.time);
  std::vector<cv::Mat> images(imageQueue_.size());
  for (size_t i = 0; i < imageQueue_.size(); i++) {
    images[i] = imageQueue_[i].image;
  }
  cv::fastNlMeansDenoisingMulti(
    images, denoised.image, centerIdx, (int)imageQueue_.size(), h_,
    templateWindowSize_, searchWindowSize_);
  return (denoised);
}
}  // namespace image_denoising
