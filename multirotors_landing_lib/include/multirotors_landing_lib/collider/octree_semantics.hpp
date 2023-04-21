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

#ifndef __OCTREE_SEMANTICS_HPP__
#define __OCTREE_SEMANTICS_HPP__

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

#include "multirotors_landing_lib/common/semantics_classes.hpp"

namespace octomap {

// Node Definition
class OcTreeNodeSemantics : public OcTreeNode {
  public:
  OcTreeNodeSemantics() : OcTreeNode(), sem_class(ml::SemClasses::kUnknown) {}

  OcTreeNodeSemantics(const OcTreeNodeSemantics& rhs)
      : OcTreeNode(rhs), sem_class(rhs.sem_class) {}

  bool operator==(const OcTreeNodeSemantics& rhs) const {
    return rhs.value == value && rhs.sem_class == sem_class;
  }

  void copyData(const OcTreeNodeSemantics& from) {
    OcTreeNode::copyData(from);
    sem_class = from.sem_class;
  }

  inline void updateNode() {
    sem_class = ml::SemClasses::kUnknown;
  }

  inline int getSemClass() const {
    return sem_class;
  }
  inline void setSemClass(const int sem_class_in) {
    sem_class = sem_class_in;
  }

  // Update occupancy and semantic class of inner nodes
  inline void updateOccupancyChildren() {
    this->setLogOdds(this->getMaxChildLogOdds());
    updateNode();
  }

  protected:
  int sem_class;  // Semantic class ID

};  // end class OcTreeNodeSemantics

// Tree definition
class OcTreeSemantics : public OccupancyOcTreeBase<OcTreeNodeSemantics> {
  public:
  OcTreeSemantics(double resolution);

  OcTreeSemantics* create() const {
    return new OcTreeSemantics(resolution);
  }

  std::string getTreeType() const {
    return "OcTreeSemantics";
  }

  virtual void updateNodeLogOdds(OcTreeNodeSemantics* node,
                                 const float& update) const;

  OcTreeNodeSemantics* integrateNodeSemantics(const OcTreeKey& key,
                                              const int sem_class);

  OcTreeNodeSemantics* integrateNodeSemantics(const point3d& p,
                                              const int sem_class);

  bool getSemClassAtPosition(const point3d& p, int& sem_class);

  protected:
  class StaticMemberInitializer {
public:
    StaticMemberInitializer() {
      OcTreeSemantics* tree = new OcTreeSemantics(0.1);
      tree->clearKeyRays();
      AbstractOcTree::registerTreeType(tree);
    }

    void ensureLinking() {}
  };
  static StaticMemberInitializer OcTreeSemanticsMemberInit;

};  // end class OcTreeSemantics

}  // end namespace octomap

#endif  //__OCTREE_SEMANTICS_HPP__
