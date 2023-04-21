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

#ifndef __COLLIDER_PLY_HPP__
#define __COLLIDER_PLY_HPP__

#include <memory>

#include "multirotors_landing_lib/collider/collider_base.hpp"
#include "multirotors_landing_lib/collider/octree_semantics.hpp"
#include "multirotors_landing_lib/common/model_defs.hpp"

namespace ml {

namespace collider {

/**
 * @brief Contains all the parameters required by this collider
 */
struct ColliderPlyParams {
  Scalar robot_dim;           //< Robot's dimension in meter
  Scalar resolution;          //< Resolution of the voxel map
  Scalar threshold_prob_occ;  //< Probability to consider a voxel occupied

  ColliderPlyParams()
      : robot_dim(0.5f), resolution(0.25f), threshold_prob_occ(0.7) {}

  ColliderPlyParams(const Scalar _robot_dim, const Scalar _resolution,
                    const Scalar _threshold_prob_occ)
      : robot_dim(_robot_dim),
        resolution(_resolution),
        threshold_prob_occ(_threshold_prob_occ) {}
};

/**
 * @brief This collider gets as input a PLY file (with color corresponding to
 * the semantic classes) and stores it in an OctoMap format.
 */
class ColliderPly final : public ColliderBase {
  public:
  /**
   * @brief Constructor
   * @param[in] params Collider parameters
   * @param[in] models_folder Folder where the ply models are stored
   */
  ColliderPly(const ColliderPlyParams &params,
              const std::string &models_folder);

  /**
   * @brief Destructor
   */
  ~ColliderPly() override;

  /**
   * @brief Function that uploads a given model in memory
   * @param[in] model_id ID of the model to be loaded
   * @attention Clears current model
   */
  void uploadModel(const ModelID model_id);

  /**
   * @brief Function to check if a position 'p' is in collision (virtual fun)
   * @param[in] p Position to check
   * @return True if in collision, False otherwise
   */
  bool isPositionInCollision(const Eigen::Vector3f &p) const override;

  /**
   * @brief Function that reads a ply mesh and stores it in OctoMap format
   * @param[in] mesh_file PLY file to read
   */
  void readMeshToOctomap(const std::string &mesh_file);

  // Return true if normal and semantic class are in a valid node in the tree
  /**
   * @brief Function that gets the semantic class at a given position
   * @param[in] p Input position
   * @param[out] sem_class Semantic class
   * @return True if the query position is in a valid node, false otherwise
   */
  bool getSemClassAtPosition(const Eigen::Vector3f &p, int &sem_class) const;

  protected:
  // Storage of map
  std::unique_ptr<octomap::OcTreeSemantics> map_;

  // Parameters and auxiliaries
  Scalar threshold_prob_occ_;
  // Auxialiaries when changing the map
  std::string models_folder_;

};  // end class

// Useful typedef
typedef std::unique_ptr<ColliderPly> ColliderPlyPtr;

}  // end namespace collider

}  // end namespace ml

#endif  // __COLLIDER_PLY_HPP__
