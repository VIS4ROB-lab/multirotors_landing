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

#include "multirotors_landing_lib/renderer/renderer_vulkan.hpp"

namespace ml {

namespace renderer {

RendererVulkan::RendererVulkan(
    const RendererParams &renderer_params,
    const collider::ColliderPlyParams &collider_params, const Matrix4 &T_BC)
    : RendererBase(renderer_params, T_BC) {
  // Initialize the actual renderer
  const std::string shader_folder =
      getenv("ML_PATH") + std::string(
                              "/multirotors_landing_lib/thirdparty/"
                              "vulkan_glasses_for_robots/shaders");
  vulkan_renderer_ = std::make_unique<vrglasses_for_robots::VulkanRenderer>(
      renderer_params.img_width, renderer_params.img_height,
      renderer_params.near, renderer_params.far, shader_folder);

  // Initialize auxiliaries
  models_folder_ = getenv("ML_PATH") + std::string("/meshes");
  buildOpenglProjectionFromIntrinsics(
      renderer_params.img_width, renderer_params.img_height,
      renderer_params.f_depth, renderer_params.f_depth, 0,
      renderer_params.cx_depth, renderer_params.cy_depth, renderer_params.near,
      renderer_params.far);

  // Initialize collider
  collider_ =
      std::make_unique<collider::ColliderPly>(collider_params, models_folder_);

  // Initialize full-size image
  rendered_rgb_.create(renderer_params.img_height, renderer_params.img_width,
                       CV_8UC3);
  rendered_semantic_.create(renderer_params.img_height,
                            renderer_params.img_width, CV_8UC1);
  rendered_depth_.create(renderer_params.img_height, renderer_params.img_width,
                         CV_32FC1);
}

RendererVulkan::~RendererVulkan() {}

void RendererVulkan::renderAtPose(const Vector3 &p_WB, const Quaternion &q_WB) {
  // Transform the pose information into a homogeneous matrix
  Matrix4 T_WB(Matrix4::Identity());
  T_WB.block<3, 3>(0, 0) = q_WB.toRotationMatrix();
  T_WB.block<3, 1>(0, 3) = p_WB;
  // Get the global viewpoint
  glm::mat4 mvp = computeMVP(T_WB * T_BC_);
  vulkan_renderer_->setCamera(mvp);

  // Rendering
  cv::Mat rgbs_map;  //< Contains RGB + Semantics
  vulkan_renderer_->renderMesh(rendered_depth_, rgbs_map);

  // Get the RGB and the Semantic images (different channels)
  cv::Mat out[] = {rendered_rgb_, rendered_semantic_};
  int from_to[] = {0, 0, 1, 1, 2, 2, 3, 3};
  cv::mixChannels(&rgbs_map, 1, out, 2, from_to, 4);
}

bool RendererVulkan::isPositionInCollision(const Vector3 &p_WB,
                                           int &sem_class) {
  collider_->getSemClassAtPosition(p_WB, sem_class);
  return collider_->isPositionInCollision(p_WB);
}

void RendererVulkan::uploadModel(const ModelID model_id) {
  // Get the path for the mesh and the texture
  std::string model_name(objFileFromId(model_id));
  std::string mesh_obj_file(models_folder_ + "/" + model_name + "/" +
                            model_name + ".obj");
  std::string texture_file(models_folder_ + "/" + model_name + "/" +
                           model_name + "_rgbs.png");

  // Uploading
  vulkan_renderer_->clearModels();
  vulkan_renderer_->loadMesh(mesh_obj_file, texture_file);
  vulkan_renderer_->noFileScene();

  // Upload to colllider
  collider_->uploadModel(model_id);
}

void RendererVulkan::buildOpenglProjectionFromIntrinsics(
    int img_width, int img_height, float alpha, float beta, float skew,
    float u0, float v0, float near, float far) {
  // These parameters define the final viewport that is rendered into by the
  // camera.
  float l = 0;
  float r = img_width;
  float b = 0;
  float t = img_height;

  // near and far clipping planes, these only matter for the mapping from
  // world-space z-coordinate into the depth coordinate for OpenGL
  float n = near;
  float f = far;

  // construct an orthographic matrix which maps from projected coordinates to
  // normalized device coordinates in the range
  // [-1, 1].  OpenGL then maps coordinates in NDC to the current viewport
  glm::mat4 ndc(0);
  ndc[0][0] = 2.f / (r - l);
  ndc[3][0] = -(r + l) / (r - l);
  ndc[1][1] = 2.f / (t - b);
  ndc[3][1] = -(t + b) / (t - b);
  ndc[2][2] = -2.f / (f - n);
  ndc[3][2] = -(f + n) / (f - n);
  ndc[3][3] = 1.0;

  // construct a projection matrix, this is identical to the projection matrix
  // computed for the intrinsic,
  // except an additional row is inserted to map the z-coordinate to OpenGL.
  // CMatrix4<T> matProjection(0);    // the 3rd column is inverted to make the
  // camera look towards +Z (instead of -Z in opengl)
  glm::mat4 matProjection = glm::mat4(0);
  matProjection[0][0] = alpha;
  matProjection[1][0] = skew;
  matProjection[2][0] = -u0;
  matProjection[1][1] = beta;
  matProjection[2][1] = -v0;
  matProjection[2][2] = (n + f);
  matProjection[3][2] = n * f;
  matProjection[2][3] = -1.0;

  // resulting OpenGL frustum is the product of the orthographic
  // mapping to normalized device coordinates and the augmented camera intrinsic
  // matrix
  perspective_ = ndc * matProjection;
  perspective_[1][1] *=
      -1;  // was originally designed for OpenGL, where the Y coordinate of the
           // clip coordinates is inverted in relation Vulkan.
}

glm::mat4 RendererVulkan::computeMVP(const Matrix4 &T_WC) const {
  const auto T_CW_cv_eigen = T_WC.inverse();
  glm::mat4 T_CW_cv_glm;
  glm::mat4 conversion_gl_cv =
      glm::mat4(1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1);

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      T_CW_cv_glm[j][i] = T_CW_cv_eigen(i, j);
    }
  }
  return perspective_ * conversion_gl_cv * T_CW_cv_glm;
}

}  // end namespace renderer

}  // end namespace ml
