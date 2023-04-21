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

#ifndef __RENDERER_VULKAN_HPP__
#define __RENDERER_VULKAN_HPP__

#include <vulkan_glasses_for_robots/vulkan_renderer.h>

#include <memory>

#include "multirotors_landing_lib/collider/collider_ply.hpp"
#include "multirotors_landing_lib/common/model_defs.hpp"
#include "multirotors_landing_lib/renderer/renderer_base.hpp"

typedef std::unique_ptr<vrglasses_for_robots::VulkanRenderer> VulkanRendererPtr;

namespace ml {

namespace renderer {

/**
 * @brief This renderer uses Vulkan to generate RGB, depth and semantic images.
 * It also check for collisions using OctoMap.
 */
class RendererVulkan final : public RendererBase {
  public:
  /**
   * @brief Constructor
   * @param[in] renderer_params Parameters to build up the renderer
   * @param[in] collider_params Parameters to build up the PLY collider
   * @param[in] T_BC transformation as homogeneous matrix in Eigen format
   *                 (default: identity)
   */
  RendererVulkan(const RendererParams& renderer_params,
                 const collider::ColliderPlyParams& collider_params =
                     collider::ColliderPlyParams(),
                 const Matrix4& T_BC = Matrix4::Identity());

  /**
   * @brief Destructor
   */
  virtual ~RendererVulkan();

  /**
   * @brief Method to render the images at a given pose
   * @param[in] p_WB Position in Eigen format (x, y, z)
   * @param[in] q_WB Orientation in Eigen format (w, x, y, z)
   */
  void renderAtPose(const Vector3& p_WB, const Quaternion& q_WB);

  /**
   * @brief Function to check if position is in collision
   * @param[in] p_WB Query position in world frame
   * @param[out] sem_class Semantic class for the query position (if the
   * query position is invalid the semantic class is 'Unknown')
   * @return True if in collision, False otherwise
   */
  bool isPositionInCollision(const Vector3& p_WB, int& sem_class) override;

  /**
   * @brief Function to load the model into the renderer
   * @param[in] model_id ID of the model
   */
  void uploadModel(const ModelID model_id);

  protected:
  /**
   * @brief Function to build the OpenGL matrix required for rendering from
   *        the camera instrinsics
   * @param[in] img_width Image width size
   * @param[in] img_height Image height size
   * @param[in] alpha Focal length
   * @param[in] beta Focal lenght
   * @param[in] skew Skew factor of the image
   * @param[in] u0 Principal point coordinate
   * @param[in] v0 Principal point coordinate
   * @param[in] near Nearest rendered distance
   * @param[in] far Farthest rendered distance
   */
  void buildOpenglProjectionFromIntrinsics(int img_width, int img_height,
                                           float alpha, float beta, float skew,
                                           float u0, float v0, float near,
                                           float far);

  /**
   * @brief Compute Model-View-Projection in OpenGL format
   * @param[in] T_WC global camera pose in Eigen format
   * @return Global camera pose in OpenGL format
   */
  glm::mat4 computeMVP(const Matrix4& T_WC) const;

  private:
  // Main rendering pipeline
  VulkanRendererPtr vulkan_renderer_;
  glm::mat4 perspective_;
  std::string models_folder_;  //< Folder where the models are stored

  // Collider
  collider::ColliderPlyPtr collider_;

};  // end class

}  // end namespace renderer

}  // end namespace ml

#endif  // __RENDERER_VULKAN_HPP__
