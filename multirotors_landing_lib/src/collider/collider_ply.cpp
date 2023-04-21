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

#include "multirotors_landing_lib/collider/collider_ply.hpp"

#include <pcl/io/ply_io.h>

namespace ml {

namespace collider {

ColliderPly::ColliderPly(const ColliderPlyParams &params,
                         const std::string &models_folder)
    : ColliderBase(params.robot_dim),
      threshold_prob_occ_(params.threshold_prob_occ),
      models_folder_(models_folder) {
  // Initialize octomap
  map_ = std::make_unique<octomap::OcTreeSemantics>(params.resolution);
}

ColliderPly::~ColliderPly() {}

void ColliderPly::uploadModel(const ModelID model_id) {
  std::string model_name(objFileFromId(model_id));
  std::string model_file(models_folder_ + "/" + model_name + "/" + model_name +
                         ".ply");
  // Make sure the octomap is empty
  map_->clear();
  // Upload new mesh
  readMeshToOctomap(model_file);
}

bool ColliderPly::isPositionInCollision(const Vector3 &p) const {
  // Now iterate over the positions in a given radius
  octomap::point3d center(p.x(), p.y(), p.z());
  octomap::point3d min(center.x() - robot_dim_, center.y() - robot_dim_,
                       center.z() - robot_dim_);
  octomap::point3d max(center.x() + robot_dim_, center.y() + robot_dim_,
                       center.z() + robot_dim_);
  for (octomap::OcTreeSemantics::leaf_bbx_iterator
           it = map_->begin_leafs_bbx(min, max),
           end = map_->end_leafs_bbx();
       it != end; ++it) {
    if (it != nullptr &&
        it->getOccupancy() > static_cast<double>(threshold_prob_occ_)) {
      return true;
    }
  }
  // In this case, we are either in unknown or free space - return that we are
  // not in collision
  return false;
}

bool ColliderPly::getSemClassAtPosition(const Eigen::Vector3f &p,
                                        int &sem_class) const {
  octomap::point3d p_octo(p.x(), p.y(), p.z());

  // Now iterate over the positions in a given radius and count the frequency
  // of the semantic classes
  octomap::point3d min(p_octo.x() - robot_dim_, p_octo.y() - robot_dim_,
                       p_octo.z() - robot_dim_);
  octomap::point3d max(p_octo.x() + robot_dim_, p_octo.y() + robot_dim_,
                       p_octo.z() + robot_dim_);

  std::map<int, size_t> sem_class_counter;
  size_t counter = 0;
  for (octomap::OcTreeSemantics::leaf_bbx_iterator
           it = map_->begin_leafs_bbx(min, max),
           end = map_->end_leafs_bbx();
       it != end; ++it) {
    if (it != nullptr &&
        it->getOccupancy() > static_cast<double>(threshold_prob_occ_)) {
      sem_class_counter[it->getSemClass()]++;
      ++counter;
    }
  }

  // If we have not found any occupied voxel, then return false
  if (counter == 0) {
    return false;
  }

  // Get most voted semantic class
  counter = 0;
  sem_class = SemClasses::kUnknown;
  for (const auto &s : sem_class_counter) {
    if (s.second > counter) {
      sem_class = s.first;
      counter = s.second;
    }
  }
  // Everything went fine
  return true;
}

void ColliderPly::readMeshToOctomap(const std::string &mesh_file) {
  // Read the mesh file
  pcl::PointCloud<pcl::PointXYZRGBNormal> pcl_mesh;
  pcl::io::loadPLYFile(mesh_file, pcl_mesh);

  // Insert in octomap
  const bool lazy_eval = true;
  for (const auto &p : pcl_mesh.points) {
    octomap::point3d p_octo(p.x, p.y, p.z);
    map_->updateNode(p_octo, true,
                     lazy_eval);  // integrate 'occupied' measurement
    // Ge the right semantic class
    const Color color(p.r, p.g, p.b);
    int sem_class = static_cast<int>(fromRgbColorToClass(color));
    // Store the occupied position alongside its semantic class
    map_->integrateNodeSemantics(p_octo, sem_class);
  }

  // This is necessary since lazy_eval is set to true.
  if (lazy_eval) {
    map_->updateInnerOccupancy();
  }
}

}  // end namespace collider

}  // end namespace ml
