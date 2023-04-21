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

#include "multirotors_landing_lib/envs/quadrotor_env.hpp"

#include <X11/Xlib.h>
#include <yaml-cpp/yaml.h>

#include <boost/filesystem.hpp>
#include <opencv2/core/eigen.hpp>

namespace ml {

QuadrotorEnv::QuadrotorEnv(const int id, const std::string& env)
    : EnvBase<renderer::RendererVulkan, QuadrotorSimplified>(id),
      fix_model_id_(false),
      reset_count_(0),
      uniform_dist_models_(0, TrainingModel3DId::kTotNumTrainingModel - 1) {
  // Check that environment variables have been declared
  if (getenv("ML_PATH") == nullptr) {
    throw std::invalid_argument("ML_PATH not declared");
  }

  // Read configuration file (all the parameters must be there!)
  if (!loadConfiguration(env)) {
    throw std::invalid_argument("Parsing of parameter file failed");
  }

  // Initialize renderer
  if (!initializeVulkanRenderer()) {
    throw std::invalid_argument("Initialization of Vulkan renderer failed");
  }

  // Initialization of auxiliaries
  this->extra_info_["success"] = 0.f;
  this->extra_info_["reason"] = DoneReason::kNone;

  // Visualization
  if (visualization_) {
    XInitThreads();
  }
}

QuadrotorEnv::~QuadrotorEnv() {
  if (this->logger_) {
    this->logger_->warn("Closing environment " + std::to_string(this->id_) +
                        "!");
  }
}

bool QuadrotorEnv::reset(Ref<Vector<>> obs) {
  // Check dimensions input w.r.t. environments
  if (obs.rows() != this->obs_dim_) {
    if (this->logger_) {
      this->logger_->error(
          "Input matrix dimensions do not match with that of the environment");
    }
    return false;
  }

  // Check if we need to change the model used by the Vulkan renderer.
  // This may be disabled for testing purposes.
  if (reset_count_ % resets_change_model_ == 0 && !fix_model_id_) {
    selectRandomModel();
  } else {
    // Increase counter and return
    ++reset_count_;
  }

  // Select the new initial location and assign it to the robot
  const Vector3 reset_pos = selectRandomPositionInModel();
  this->robot_->setState(reset_pos);

  // Reset step (and other auxiliaries)
  this->step_ = 0;
  this->extra_info_["success"] = 0.f;
  this->extra_info_["reason"] = DoneReason::kNone;

  // Get observation
  return getObs(obs);
}

Scalar QuadrotorEnv::step(const Ref<Vector<>> act, Ref<Vector<>> obs) {
  // Calculate the reward from the episode
  Scalar reward = 0.;

  // Feed the action to the robot and get the updated position. Here we fix
  // the orientation to be constant (assuming a downlooking camera). This
  // works only for the simplified quadrotor state.
  const Quaternion orientation(Quaternion::Identity());

  // Render images
  const Vector3 position = this->robot_->getState();
  this->renderer_->renderAtPose(position, orientation);

  // Apply the action to the robot
  this->robot_->run(static_cast<int>(act[0]), this->sim_dt_);

  // Get the images from the renderer
  getInput();

  // Resize the images
  // NOTICE: the depth is type 32FC1, while semantics is 8UC1, so in one case
  // we interpolate, in the other we get the nearest neighbour
  const int fx = 0, fy = 0;
  cv::resize(depth_, depth_, cv::Size(img_width_, img_height_));
  cv::resize(semantics_, semantics_, cv::Size(img_width_, img_height_), fx, fy,
             cv::INTER_NEAREST);

  // Visualization
  if (visualization_) {
    const auto id_str(std::to_string(this->id_));
    this->renderer_->showImages(id_str);
  }

  // Get the observation
  bool success = getObs(obs);
  if (!success) {
    if (this->logger_) {
      this->logger_->error(
          "Getting the observation failed. Returning 0 reward");
      return reward;
    }
  }

  // Compute the rewards

  // Step reward
  const Scalar time_reward = reward_params_.step;

  // Semantic reward
  const Scalar sem_reward = getStepRewardFromSemantics();

  // Collision reward
  double min, max;
  cv::minMaxLoc(depth_, &min, &max);
  Scalar coll_reward = static_cast<float>(min) > collision_params_.min_dist
                           ? reward_params_.coll_survival
                           : -1.f / (std::fabs(static_cast<float>(min) -
                                               collision_params_.robot_radius) +
                                     1e-3f);

  // Action reward
  Scalar reward_action = 0.f;
  if (std::fabs(act[0]) < 1e-6f) {
    reward_action = reward_params_.stand_still;
  } else if (act[0] < 9 && act[0] > 0) {
    reward_action = reward_params_.lateral_mov;
  } else if (act[0] >= 9) {
    reward_action = reward_params_.down_mov;
  }

  // Total reward
  reward = time_reward + reward_params_.sem_weight * sem_reward +
           reward_params_.coll_weight * coll_reward + reward_action;
  return reward;
}

std::unordered_map<SemClasses, size_t> QuadrotorEnv::getFrequenciesClasses()
    const {
  // Get semantic image size
  int nRows = semantics_.rows;
  int nCols = semantics_.cols * semantics_.channels();
  if (semantics_.isContinuous()) {
    nCols *= nRows;
    nRows = 1;
  }

  // Iterate over the pixel of the semantic image and get the frequencies of
  // the pixel and their associated semantic class
  std::unordered_map<SemClasses, size_t> freq_classes;
  for (int i = 0; i < nRows; ++i) {
    const uchar* p = semantics_.ptr<uchar>(i);
    for (int j = 0; j < nCols; ++j) {
      int px_val = static_cast<int>(p[j]);
      freq_classes[fromGrayIntensityToClass(px_val)]++;
    }
  }
  return freq_classes;
}

void QuadrotorEnv::selectRandomModel() {
  // Select model (make sure it is not the same model already in memory)
  while (true) {
    ModelID candidate_id =
        static_cast<ModelID>(uniform_dist_models_(random_gen_));
    if (model_id_ != candidate_id) {
      model_id_ = candidate_id;
      break;
    }
  }

  // Upload model
  uploadModel();

  // Update counter
  reset_count_ = 1;
}

Vector3 QuadrotorEnv::selectRandomPositionInModel() {
  // Here we first select the bounding box across all the possibile ones, and
  // then sample a position (x, y, z)
  // Note: Here we are not checking if the model exists - assuming that the data
  // are consistent (the correspondence model <-> bounding box is always valid)
  const auto& bounding_boxes = reset_bbs_.find(model_id_)->second;

  std::uniform_real_distribution<Scalar> uniform_dist_pos{-1.0, 1.0};
  std::uniform_int_distribution<size_t> uniform_dist_bbs(
      0, bounding_boxes.size() - 1);

  const auto sample = uniform_dist_bbs(random_gen_);
  Vector3 reset_pos_random;
  for (uint i = 0; i < 3; ++i) {
    reset_pos_random[i] =
        bounding_boxes[sample].center[i] +
        uniform_dist_pos(random_gen_) * bounding_boxes[sample].dimension[i];
  }
  return reset_pos_random;
}

bool QuadrotorEnv::storeBoundingBoxesReset() {
  // Folder where reset information is stored
  const auto& reset_info_folder =
      getenv("ML_PATH") +
      std::string("/multirotors_landing_lib/config/models_reset/");

  // Iterate over the models, and read the corresponding bounding box info
  const ModelID num_tot_models = TrainingModel3DId::kTotNumTrainingModel +
                                 TestingModel3DId::kTotNumTestingModel;
  for (ModelID id = 0; id < num_tot_models; ++id) {
    const std::string& model_name = objFileFromId(id);
    const std::string& file_name =
        reset_info_folder + model_name + std::string(".yaml");

    if (!boost::filesystem::exists(file_name)) {
      this->logger_->error("Reset information for model %s does not exist",
                           model_name);
      return false;
    }

    YAML::Node cfg = YAML::LoadFile(file_name);
    const size_t n_reset_pos = cfg["n_reset_pos"].as<size_t>();
    reset_bbs_[id] = std::vector<ResetBoundingBox>(n_reset_pos);

    for (size_t j = 0; j < n_reset_pos; ++j) {
      reset_bbs_[id][j].center =
          Vector3(cfg["reset_pos"]["center_" + std::to_string(j)]
                      .as<std::vector<float>>()
                      .data());
      reset_bbs_[id][j].dimension =
          Vector3(cfg["reset_pos"]["dim_" + std::to_string(j)]
                      .as<std::vector<float>>()
                      .data());
    }
  }

  return true;
}

void QuadrotorEnv::uploadModel() {
  if (this->logger_) {
    this->logger_->info("Loading: " + ml::objFileFromId(model_id_));
  }

  const auto start = std::chrono::high_resolution_clock::now();
  renderer_->uploadModel(model_id_);
  const auto end = std::chrono::high_resolution_clock::now();
  const double elapsed =
      std::chrono::duration_cast<std::chrono::seconds>(end - start).count();

  if (this->logger_) {
    this->logger_->info("Elapsed time: %1.1f s", elapsed);
  }
}

Scalar QuadrotorEnv::getStepRewardFromSemantics() const {
  std::unordered_map<SemClasses, size_t> freq_classes = getFrequenciesClasses();
  float nTotPx = static_cast<float>(semantics_.rows * semantics_.cols *
                                    semantics_.channels());

  Scalar reward = 0.0;
  for (const auto& f : freq_classes) {
    float frequency = static_cast<float>(f.second) / nTotPx;
    if (f.first == SemClasses::kTerrain) {
      reward += reward_params_.sem_good * frequency;
    } else {
      reward -= reward_params_.sem_bad * frequency;
    }
  }
  return reward;
}

SemClasses QuadrotorEnv::getDominantClass(
    const std::unordered_map<SemClasses, size_t>& freq_classes,
    const int num_tot_px) const {
  SemClasses dominant = SemClasses::kUnknown;
  float max_coverage = 0.f;
  const float num_tot_px_f = static_cast<float>(num_tot_px);

  for (const auto& f : freq_classes) {
    float coverage = static_cast<float>(f.second) / num_tot_px_f;
    if (coverage > max_coverage) {
      max_coverage = coverage;
      dominant = f.first;
    }
  }

  return dominant;
}

bool QuadrotorEnv::checkTerminalConditionClasses(Scalar& reward) {
  // Iterate over the map, and if we have a sufficient coverage with one class,
  // then return true/false (depending on class). Wait for a minumum number of
  // steps before checking the classes (otherwise binary case always lose)

  constexpr float max_coverage = 0.90f;
  int nTotPx;
  std::unordered_map<SemClasses, size_t> freq_classes;

  // Get classes frequencies
  freq_classes = getFrequenciesClasses();
  nTotPx = semantics_.rows * semantics_.cols * semantics_.channels();

  // Check coverage
  for (const auto& f : freq_classes) {
    float coverage = static_cast<float>(f.second) / static_cast<float>(nTotPx);
    if (coverage >= max_coverage) {
      if (/*f.first == SemClasses::kPavement || */
          f.first == SemClasses::kTerrain) {
        reward = reward_params_.landing_good_class;
        this->extra_info_["success"] = 1.f;
        this->extra_info_["reason"] = DoneReason::kSemantics;
      } else {
        reward = reward_params_.landing_bad_class;
        this->extra_info_["success"] = 0.f;
        this->extra_info_["reason"] = DoneReason::kSemantics;
      }
      return true;
    }
  }

  // No max coverage
  reward = 0;
  return false;
}

bool QuadrotorEnv::getObs(Ref<Vector<>> obs) {
  // Feed the depth and semantic information into the vector
  if (depth_.empty() || semantics_.empty()) {
    return false;
  }

  // Process semantics
  Eigen::MatrixXf im_semantic;
  Eigen::VectorXf iv_semantic;
  cv::cv2eigen(semantics_, im_semantic);
  // To solve conversion problem eigen <-> numpy. This is taking 80% of the
  // total time of the function
  im_semantic.transposeInPlace();
  iv_semantic = Eigen::Map<Eigen::VectorXf>(
      im_semantic.data(), im_semantic.cols() * im_semantic.rows());

  // Process depth
  Eigen::MatrixXf im_depth;
  Eigen::VectorXf iv_depth;
  cv::cv2eigen(depth_, im_depth);
  // To solve conversion problem eigen <-> numpy. This is taking 80% of the
  // total time of the function
  cv::cv2eigen(depth_, im_depth);
  im_depth.transposeInPlace();
  iv_depth = Eigen::Map<Eigen::VectorXf>(im_depth.data(),
                                         im_depth.cols() * im_depth.rows());

  // Generate final output
  obs << iv_depth, iv_semantic;

  return true;
}

bool QuadrotorEnv::isTerminalState(Scalar& reward) {
  // Get current robot's state
  auto robot_state = this->robot_->getState();

  // Check if we reached target height
  if (reward_params_.target_height > 0 &&
      robot_state[2] < reward_params_.target_height) {
    this->extra_info_["reason"] = DoneReason::kTargetHeight;

    // Decide if it is a success depending on the dominant semantic class
    const auto freq_classes = getFrequenciesClasses();
    const int nTotPx = this->semantics_.rows * this->semantics_.cols *
                       this->semantics_.channels();
    if (getDominantClass(freq_classes, nTotPx) == SemClasses::kTerrain) {
      this->extra_info_["success"] = 1.f;
      reward = reward_params_.landing_good_class;
    } else {
      this->extra_info_["success"] = 0.f;
      reward = reward_params_.landing_bad_class;
    }

    // Info
    if (this->logger_) {
      std::cout << "\033[1;32mTerminal case: TARGET HEIGHT\033[0m" << std::endl;
      std::cout << "   => total: " << reward << "\n" << std::endl;
      std::cout << "   => classes distribution: " << std::endl;
      for (const auto& f : freq_classes) {
        std::cout << "     - Class " << stringFromClassId(f.first) << ": "
                  << static_cast<float>(f.second) / static_cast<float>(nTotPx) *
                         100.f
                  << "%" << std::endl;
      }
    }
    return true;
  }

  // Check for collisions
  int sem_class;
  if (this->renderer_->isPositionInCollision(robot_state, sem_class)) {
    // Get the reward associated to semantics
    Scalar reward_sem;
    // NOTE: Here we consider terrain the only valid class to land on
    if (sem_class == SemClasses::kTerrain) {
      reward_sem = reward_params_.landing_good_class;
      this->extra_info_["success"] = 1.f;
      this->extra_info_["reason"] = DoneReason::kSemantics;
    } else {
      reward_sem = reward_params_.landing_bad_class;
      this->extra_info_["success"] = 0.f;
      this->extra_info_["reason"] = DoneReason::kCollision;
    }
    // Total reward
    reward = reward_sem;

    // Info
    if (this->logger_) {
      std::cout << "\033[1;31mTerminal case: COLLISION\033[0m" << std::endl;
      std::cout << "- Reward semantics: " << reward_sem << std::endl;
      std::cout << "   => total: " << reward << "\n" << std::endl;
    }
    return true;
  }

  // Check we have (almost) just one semantic class in the image
  if (reward_params_.use_early_stop_semantics &&
      checkTerminalConditionClasses(reward)) {
    // Info
    if (this->logger_) {
      std::cout << "\033[1;32mTerminal case: SEMANTIC CLASSES\033[0m"
                << std::endl;
      std::cout << "   => total: " << reward << "\n" << std::endl;
    }
    return true;
  }

  // Number of iterations check
  if (this->sim_dt_ * this->step_ >= this->max_t_) {
    reward = reward_params_.max_time;
    this->extra_info_["success"] = 0.f;
    this->extra_info_["reason"] = DoneReason::kMaxTime;
    // Info
    if (this->logger_) {
      std::cout << "\033[1;31mTerminal case: MAX TIME\033[0m" << std::endl;
      std::cout << "   => total: " << reward << "\n" << std::endl;
    }
    return true;
  }

  // Check if we got out of environment limits
  if (robot_state[0] < this->env_limits_[0] ||
      robot_state[0] > this->env_limits_[1] ||
      robot_state[1] < this->env_limits_[2] ||
      robot_state[1] > this->env_limits_[3] ||
      robot_state[2] < this->env_limits_[4] ||
      robot_state[2] > this->env_limits_[5]) {
    reward = reward_params_.out_of_bounds;
    this->extra_info_["success"] = 0.f;
    this->extra_info_["reason"] = DoneReason::kBoundaries;

    // Info
    if (this->logger_) {
      std::cout << "\033[1;31mTerminal case: BOUNDARIES\033[0m" << std::endl;
      std::cout << "   => total: " << reward << "\n" << std::endl;
    }

    return true;
  }

  // If we are not done, then continue and increment the step iterator
  ++this->step_;
  return false;
}

bool QuadrotorEnv::getRobotState(Ref<Vector<>> state) {
  state << this->robot_->getState();
  return true;
}

bool QuadrotorEnv::setModelId(Ref<Vector<>> model_id) {
  // Auxiliary
  const bool update = model_id_ != ModelID(model_id[0]);
  model_id_ = ModelID(model_id[0]);
  fix_model_id_ = true;

  // Update model only if necessary
  if (update) {
    uploadModel();
  }

  return true;
}

bool QuadrotorEnv::loadConfiguration(const std::string& env) {
  // Load YAML file
  const auto& file_path = getenv("ML_PATH") +
                          std::string("/multirotors_landing_lib/config/") +
                          env + std::string(".yaml");
  YAML::Node cfg = YAML::LoadFile(file_path);

  // Set-up commons
  if (cfg["common"]) {
    const auto& common_cfg = cfg["common"];
    visualization_ = common_cfg["visualization"].as<bool>();
    const bool verbose = common_cfg["verbose"].as<bool>();
    if (verbose) {
      this->logger_ = std::make_unique<ml::Logger>(env);
    }
  } else {
    // Here we just don't initialize the logger, and set visualization to false
    // (assuming that the user doesn't want it)
    visualization_ = false;
  }

  // Read parameters
  try {
    if (cfg["training_env"]) {
      // Cache the configuration
      const auto& training_cfg = cfg["training_env"];

      // Type of input (whether from ground truth or deep learning methods)
      type_input_ = training_cfg["type_input"].as<std::string>();
      assert(type_input_ == GROUND_TRUTH or type_input_ == DEEP_LEARNING);

      // Simulation parameters
      this->sim_dt_ = training_cfg["sim_dt"].as<Scalar>();
      this->max_t_ = training_cfg["max_t"].as<Scalar>();

      // State spaces
      this->act_dim_ = training_cfg["act_dim"].as<int>();
      img_width_ = training_cfg["img_width"].as<int>();
      img_height_ = training_cfg["img_height"].as<int>();
      const int img_channels = training_cfg["img_channels"].as<int>();
      assert(img_channels == 1 || img_channels == 2 || img_channels == 3);
      this->obs_dim_ = img_width_ * img_height_ * img_channels;

      // Reward
      const auto& reward_cfg = training_cfg["reward"];
      if (reward_cfg) {
        // Config
        reward_params_.use_early_stop_semantics =
            reward_cfg["use_early_stop_semantics"].as<bool>();

        // Terminal
        reward_params_.landing_good_class =
            reward_cfg["landing_good_class"].as<Scalar>();
        reward_params_.landing_bad_class =
            reward_cfg["landing_bad_class"].as<Scalar>();
        reward_params_.max_time = reward_cfg["max_time"].as<Scalar>();
        reward_params_.target_height = reward_cfg["target_height"].as<Scalar>();

        // Step
        reward_params_.step = reward_cfg["step"].as<Scalar>();
        reward_params_.sem_good = reward_cfg["sem_good"].as<Scalar>();
        reward_params_.sem_bad = reward_cfg["sem_bad"].as<Scalar>();

        // Penalization lateral movements
        reward_params_.lateral_mov = reward_cfg["lateral_mov"].as<Scalar>();
        // Penalization hovering
        reward_params_.stand_still = reward_cfg["stand_still"].as<Scalar>();
        // Incentivation landing movements
        reward_params_.down_mov = reward_cfg["down_mov"].as<Scalar>();

        // Collision
        reward_params_.coll_survival = reward_cfg["coll_survival"].as<Scalar>();

        // Environment limits
        reward_params_.out_of_bounds = reward_cfg["out_of_bounds"].as<Scalar>();

        // Weights
        reward_params_.sem_weight = reward_cfg["sem_weight"].as<Scalar>();
        reward_params_.coll_weight = reward_cfg["coll_weight"].as<Scalar>();
        reward_params_.distance_weight =
            reward_cfg["distance_weight"].as<Scalar>();
      } else {
        // No reward config found
        if (this->logger_)
          this->logger_->error("No reward parameters found");
        return false;
      }

      // Collision information
      auto collision_cfg = training_cfg["collision"];
      if (collision_cfg) {
        collision_params_.min_dist = collision_cfg["min_dist"].as<Scalar>();
        collision_params_.robot_radius =
            collision_cfg["robot_radius"].as<Scalar>();
      } else {
        // No collision info found
        if (this->logger_)
          this->logger_->error("No collision parameters found");
        return false;
      }

      // Environment limits
      this->env_limits_[0] = training_cfg["env_lim_min_x"].as<Scalar>();
      this->env_limits_[1] = training_cfg["env_lim_max_x"].as<Scalar>();
      this->env_limits_[2] = training_cfg["env_lim_min_y"].as<Scalar>();
      this->env_limits_[3] = training_cfg["env_lim_max_y"].as<Scalar>();
      this->env_limits_[4] = training_cfg["env_lim_min_z"].as<Scalar>();
      this->env_limits_[5] = training_cfg["env_lim_max_z"].as<Scalar>();

    } else {
      // No training config foud
      if (this->logger_)
        this->logger_->error("No training environment parameters found");
      return false;
    }

  } catch (YAML::Exception& ex) {
    // Some parameters were not found
    std::cout << ex.what() << std::endl;
    return false;
  }

  // Initialization of quadrotor
  try {
    this->robot_ = std::make_shared<QuadrotorSimplified>(cfg);
  } catch (YAML::Exception& /*ex*/) {
    // Quadrotor parameters not found
    if (this->logger_)
      this->logger_->error("Quadrotor parameter parsing failed");
    return false;
  }

  if (this->logger_)
    this->logger_->info("Quadrotor initialized");

  // Everything ok
  if (this->logger_) {
    this->logger_->info("Environment " + std::to_string(this->id_) +
                        " set up!");
  }
  return true;
}

bool QuadrotorEnv::initializeVulkanRenderer() {
  // Load YAML file
  const auto& file_path =
      getenv("ML_PATH") +
      std::string("/multirotors_landing_lib/config/renderer/vulkan.yaml");
  YAML::Node cfg = YAML::LoadFile(file_path);

  // Read parameters
  renderer::RendererParams renderer_params;
  collider::ColliderPlyParams collider_param;
  Matrix4 T_BC(Matrix4::Identity());

  try {
    // Renderer parameters
    renderer_params.img_width = cfg["img_width"].as<int>();
    renderer_params.img_height = cfg["img_height"].as<int>();

    renderer_params.cx_depth = cfg["cx_depth"].as<Scalar>();
    renderer_params.cy_depth = cfg["cy_depth"].as<Scalar>();
    renderer_params.f_depth = cfg["f_depth"].as<Scalar>();

    // PLY collider parameters
    const auto cfg_collider = cfg["collider"];
    collider_param.robot_dim = cfg_collider["robot_dim"].as<Scalar>();
    collider_param.resolution = cfg_collider["resolution"].as<Scalar>();
    collider_param.threshold_prob_occ =
        cfg_collider["threshold_prob_occ"].as<Scalar>();

    // Extrinsics camera calibration
    T_BC.block<3, 1>(0, 3) = Vector3(0.015f, 0.055f, 0.0065f);

    const Scalar pitch =
        cfg["pitch"].as<Scalar>() * static_cast<Scalar>(M_PI / 180.0);
    Matrix<3, 3> R_pitch(Eigen::AngleAxisf(-pitch, Vector3::UnitX()));
    Matrix<3, 3> R_SC0(Matrix<3, 3>::Zero());
    R_SC0(0, 2) = 1.0;
    R_SC0(1, 0) = -1.0;
    R_SC0(2, 1) = -1.0;
    T_BC.block<3, 3>(0, 0) = R_SC0 * R_pitch;

    // Read parameters associated to Vulkan models
    resets_change_model_ = cfg["resets_change_model"].as<size_t>();

    // Set to invalid model at first (it will be update at first reset)
    model_id_ = TrainingModel3DId::kTotNumTrainingModel +
                TestingModel3DId::kTotNumTestingModel;

  } catch (const YAML::Exception& /*ex*/) {
    return false;
  }

  // Initialize the renderer
  renderer_ = std::make_shared<renderer::RendererVulkan>(renderer_params,
                                                         collider_param, T_BC);

  // Pre-allocate the bounding boxes used for environment reset
  if (!storeBoundingBoxesReset()) {
    throw std::runtime_error("Reset file not found");
  } else if (this->logger_) {
    this->logger_->info("Reset information parsed correctly");
  }

  return true;
}

void QuadrotorEnv::getInput() {
  if (type_input_ == GROUND_TRUTH) {
    depth_ = this->renderer_->getDepthImage();
    semantics_ = this->renderer_->getSemanticImage();
  } else {
    // Not implemented yet
    throw std::runtime_error("Not implemented yet");
  }
}

}  // end namespace ml
