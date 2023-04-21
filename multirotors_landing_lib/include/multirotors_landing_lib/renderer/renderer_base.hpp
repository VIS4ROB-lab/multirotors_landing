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

#ifndef __RENDERER_BASE_HPP__
#define __RENDERER_BASE_HPP__

#include <opencv2/opencv.hpp>

#include "multirotors_landing_lib/common/typedefs.hpp"

namespace ml {

namespace renderer {

/**
 * @brief Contains all the parameters required by the renderer
 */
struct RendererParams {
  // Size of the images
  int img_width;
  int img_height;

  // Rendering interval
  Scalar near;  //< Nearest rendered distance
  Scalar far;   //< Farthest rendered distance

  // Depth Parameters
  Scalar cx_depth;  //< Principal point X coordinate
  Scalar cy_depth;  //< Principal point Y coordinate
  Scalar f_depth;   //< Focal length

  RendererParams(const int _img_width, const int _img_height,
                 const Scalar _near, const Scalar _far, const Scalar _cx,
                 const Scalar _cy, const Scalar _f)
      : img_width(_img_width),
        img_height(_img_height),
        near(_near),
        far(_far),
        cx_depth(_cx),
        cy_depth(_cy),
        f_depth(_f) {}

  // Initialization with default values
  RendererParams()
      : img_width(752),
        img_height(480),
        near(0.05f),
        far(250.f),
        cx_depth(376.5f),
        cy_depth(240.5f),
        f_depth(455.f) {}
};

/**
 * @brief Base class for the renderer - it outputs RGB, depth and semantics
 * from a give pose, and can check if a given position is in collision
 */
class RendererBase {
  public:
  /**
   * @brief Constructor
   * @param[in] params Parameters to build up the renderer
   * @param[in] T_BC transformation as homogeneous matrix in Eigen format
   *                 (default: identity)
   */
  RendererBase(const RendererParams& params,
               const Matrix4& T_BC = Matrix4::Identity());

  /**
   * @brief Destructor
   */
  virtual ~RendererBase();

  /**
   * @brief Function to update the parameters
   * @param[in] params Set of parameters
   */
  inline void setParameters(const RendererParams& params) {
    params_ = params;
  }

  /**
   * @brief Function to set the extrinsic calibration body-camera
   * @param[in] T_BC transformation as homogeneous matrix in Eigen format
   */
  inline void setExtrinsicTransformation(const Matrix4& T_BC) {
    T_BC_ = T_BC;
  }

  /**
   * @brief Get the RGB image rendered by the pipeline
   * @return RGB image in OpenCV format
   */
  inline cv::Mat getRgbImage() const {
    return rendered_rgb_;
  }

  /**
   * @brief Get the depth image rendered by the pipeline
   * @return Depth image in OpenCV format
   */
  inline cv::Mat getDepthImage() const {
    return rendered_depth_;
  }

  /**
   * @brief Get the semantic image rendered by the pipeline
   * @return Semantic image in OpenCV format
   */
  inline cv::Mat getSemanticImage() const {
    return rendered_semantic_;
  }

  /**
   * @brief Method to render the images at a given pose
   * @param[in] p_WB Position in Eigen format (x, y, z)
   * @param[in] q_WB Orientation in Eigen format (w, x, y, z)
   */
  virtual void renderAtPose(const Vector3& p_WB, const Quaternion& q_WB) = 0;

  /**
   * @brief Function to check if position is in collision
   * @param[in] p_WB Query position in world frame
   * @param[out] sem_class Semantic class for the query position (if the
   * query position is invalid the semantic class is 'Unknown')
   * @return True if in collision, False otherwise
   */
  virtual bool isPositionInCollision(const Vector3& p_WB, int& sem_class) = 0;

  /**
   * @brief Function to reset renderer's properties
   */
  void reset();

  /**
   * @brief Debug method to show the rendered images (via imshow of OpenCV)
   * @param[in] window Name of the window where image is displayed
   */
  virtual void showImages(const std::string& window) const;

  protected:
  // Output
  cv::Mat rendered_rgb_;
  cv::Mat rendered_depth_;
  cv::Mat rendered_semantic_;

  // Parameters
  RendererParams params_;

  // Extrinsics transformation (from body to camera)
  Matrix4 T_BC_;  //< Homogeneous transformation

};  // end class

}  // end namespace renderer

}  // end namespace ml

#endif  // __RENDERER_BASE_HPP__
