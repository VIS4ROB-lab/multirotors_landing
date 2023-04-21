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

#include "multirotors_landing_lib/collider/octree_semantics.hpp"

namespace octomap {

OcTreeSemantics::OcTreeSemantics(double resolution)
    : OccupancyOcTreeBase<OcTreeNodeSemantics>(resolution) {
  OcTreeSemanticsMemberInit.ensureLinking();
}

void OcTreeSemantics::updateNodeLogOdds(OcTreeNodeSemantics* node,
                                        const float& update) const {
  OccupancyOcTreeBase<OcTreeNodeSemantics>::updateNodeLogOdds(node, update);
  node->updateNode();
}

OcTreeNodeSemantics* OcTreeSemantics::integrateNodeSemantics(
    const OcTreeKey& key, const int sem_class) {
  OcTreeNodeSemantics* n = search(key);
  if (n != nullptr) {
    n->setSemClass(sem_class);
  }
  return n;
}

OcTreeNodeSemantics* OcTreeSemantics::integrateNodeSemantics(
    const point3d& p, const int sem_class) {
  OcTreeKey key;
  if (!this->coordToKeyChecked(p, key))
    return nullptr;
  return integrateNodeSemantics(key, sem_class);
}

bool OcTreeSemantics::getSemClassAtPosition(const point3d& p, int& sem_class) {
  OcTreeKey key;
  if (!this->coordToKeyChecked(p, key))
    return false;
  OcTreeNodeSemantics* n = search(key);
  if (n != nullptr) {
    sem_class = n->getSemClass();
    return true;
  }
  return false;
}

OcTreeSemantics::StaticMemberInitializer
    OcTreeSemantics::OcTreeSemanticsMemberInit;

}  // namespace octomap
