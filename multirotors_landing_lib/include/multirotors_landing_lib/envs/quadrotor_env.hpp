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

#ifndef __QUADROTOR_ENV_HPP__
#define __QUADROTOR_ENV_HPP__

// Common defs
#include "multirotors_landing_lib/common/common_defs.hpp"
#include "multirotors_landing_lib/common/model_defs.hpp"
#include "multirotors_landing_lib/envs/env_base.hpp"

// Renderers
#include "multirotors_landing_lib/renderer/renderer_base.hpp"
#include "multirotors_landing_lib/renderer/renderer_vulkan.hpp"

// Quadrotor dynamics
#include "multirotors_landing_lib/robots/quadrotor_simplified.hpp"

// Type of inputs to be fed to the RL policy
#define GROUND_TRUTH "ground_truth"
#define DEEP_LEARNING "deep_learning"

namespace ml {

class QuadrotorEnv final
    : public EnvBase<renderer::RendererVulkan, QuadrotorSimplified> {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Constructor
   * @param id ID of the environment
   * @param env Name of the environment
   */
  QuadrotorEnv(const int id, const std::string &env);

  /**
   * @brief Destructor
   */
  ~QuadrotorEnv();

  /**
   * @brief Resets the environment to its starting state
   * @param[out] obs Observation at reset state
   * @return True if reset is successful, False otherwise
   */
  bool reset(Ref<Vector<>> obs) override;

  /**
   * @brief Takes one step in the environment
   * @param[in] act Action for the step
   * @param[out] obs Observation after having taken the step
   * @return Reward value
   */
  Scalar step(const Ref<Vector<>> act, Ref<Vector<>> obs) override;

  /**
   * @brief Get current observation
   * @param[out] obs Current observation from the environment
   * @return True if reset is successful, False otherwise
   */
  bool getObs(Ref<Vector<>> obs) override;

  /**
   * @brief Checks if the current state if a terminal state
   * @param[out] reward Terminal reward (0 if state is not terminal)
   * @return True if current state is terminal state, False otherwise
   */
  bool isTerminalState(Scalar &reward) override;

  /**
   * @brief Gets the robot state
   * @param[out] state State of the robot
   * @return True if successful, False otherwise
   */
  bool getRobotState(Ref<Vector<>> state) override;

  /**
   * @brief Sets the ID of the model used in the renderer
   * @param[in] model_id Model ID to set
   * @return True if setting was successful, False otherwise
   */
  bool setModelId(Ref<Vector<>> model_id);

  protected:
  /**
     * @brief Parse the YAML configuration and stores parameters to memory
     * @param[in] env Name of the environment (initialize logging)
     * @return True if all parameters are parsed correctly, False otherwise
     */
  bool loadConfiguration(const std::string &env);

  /**
   * @brief Parse the YAML configuration and stores renderer's parameters to memory
   * @return True if all parameters are parsed correctly, False otherwise
   */
  bool initializeVulkanRenderer();

  /**
   * @brief Function to get the input images (from ground truth or deep learn.)
   */
  void getInput();

  /**
   * @brief Computes the rewards from the semantic classes in the field of view
   * @return Reward value
   */
  Scalar getStepRewardFromSemantics() const;

  /**
   * @brief Get the most visible semantic class (w.r.t. number of pixels)
   * @param[in] freq_classes Hash table with number of pixels associated to each semantic class
   * @param[in] num_tot_px Total number of pixels in the semantic image
   * @return ID of the dominant semantic mask
   */
  SemClasses getDominantClass(
      const std::unordered_map<SemClasses, size_t> &freq_classes,
      const int num_tot_px) const;

  /**
   * @brief Checks if we have enough coverage (90%) of the semantic mask by a semantic class. If so, we consider the episode done
   * @param[out] reward The reward calculated from the semantic information. If not done, reward is 0
   * @return True if episode can be considered terminated, False otherwise
   */
  bool checkTerminalConditionClasses(Scalar &reward);

  /**
   * @brief Get the relative frequencies for each semantic class in the semantic
   * mask
   * @return Look-up table where keys are the ID of the semantic classes and the
   * values the number of pixels with that class
   */
  std::unordered_map<SemClasses, size_t> getFrequenciesClasses() const;

  /**
   * @brief Function to select a random model and load it
   */
  void selectRandomModel();

  /**
   * @brief Function to select an initial position in the current model. It uses
   * the pre-allocated reset bounding boxes
   */
  Vector3 selectRandomPositionInModel();

  /**
   * @brief Function to pre-allocate bounding boxes for reset per each model by
   * reading them from configuration files
   * @return True if reset is successful, False otherwise
   */
  bool storeBoundingBoxesReset();

  /**
   * @brief Uploads current model ID stored in memory
   */
  void uploadModel();

  private:
  // State space
  std::string type_input_;  //< Either ground_truth or deep_learning
  cv::Mat depth_, semantics_;

  // Main parameters
  RewardParams reward_params_;
  CollisionParams collision_params_;

  // This is the image size for the state space
  int img_width_, img_height_;

  // Auxiliaries
  bool visualization_;  //< Whether to show the images at every step

  /// Vulkan Model management ///
  // Auxiliaries
  ModelID model_id_;   //< Current ID of the model used in Vulkan
  bool fix_model_id_;  //< Whether to use only the fixed ID (for testing)
  size_t resets_change_model_;  //< Number of resets to change the model
  size_t reset_count_;          //< Counter of number of resets

  // Utilities for reset: store the possibile areas for environment resets
  // in a look up table, after reading them from file. The resets areas are
  // stored as bounding boxes.
  struct ResetBoundingBox {
    Vector3 center;
    Vector3 dimension;

    ResetBoundingBox() {}
    ResetBoundingBox(const Vector3 &_center, const Vector3 &_dimension)
        : center(_center), dimension(_dimension) {}
  };
  std::unordered_map<ModelID, std::vector<ResetBoundingBox>> reset_bbs_;

  // Random number generator to select model ID
  std::uniform_int_distribution<ModelID> uniform_dist_models_;
  std::random_device rd_;
  std::mt19937 random_gen_{rd_()};

};  // end class

}  // end namespace ml

#endif  // __QUADROTOR_ENV_HPP__
