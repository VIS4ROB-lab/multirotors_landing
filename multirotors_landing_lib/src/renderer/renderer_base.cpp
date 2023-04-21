/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022,
 *  ETH Zurich - V4RL, Department of Mechanical and Process Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors: Luca Bartolomei
 *********************************************************************/

#include "multirotors_landing_lib/renderer/renderer_base.hpp"

#include "multirotors_landing_lib/common/semantics_classes.hpp"

namespace ml {

namespace renderer {

RendererBase::RendererBase(const RendererParams& params, const Matrix4& T_BC)
    : params_(params), T_BC_(T_BC) {
  // Initialize output images
  rendered_rgb_ =
      cv::Mat::zeros(params_.img_height, params_.img_width, CV_8UC3);
  rendered_depth_ =
      cv::Mat::zeros(params_.img_height, params_.img_width, CV_32FC1);
  rendered_semantic_ =
      cv::Mat::zeros(params_.img_height, params_.img_width, CV_8UC1);
}

RendererBase::~RendererBase() {}

void RendererBase::reset() {
  rendered_rgb_ =
      cv::Mat::zeros(params_.img_height, params_.img_width, CV_8UC3);
  rendered_depth_ =
      cv::Mat::zeros(params_.img_height, params_.img_width, CV_32FC1);
  rendered_semantic_ =
      cv::Mat::zeros(params_.img_height, params_.img_width, CV_8UC1);
}

void RendererBase::showImages(const std::string& window) const {
  cv::imshow("RGB " + window, rendered_rgb_);
  cv::imshow("Depth " + window, rendered_depth_ / 255);
  cv::imshow("Semantics " + window, getColorSemantic(rendered_semantic_));
  cv::waitKey(1);
}

}  // end namespace renderer

}  // end namespace ml
