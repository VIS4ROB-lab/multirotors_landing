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

#ifndef __MODEL_DEFS_HPP__
#define __MODEL_DEFS_HPP__

#include <cstddef>
#include <stdexcept>
#include <string>

namespace ml {

typedef size_t ModelID;

enum TrainingModel3DId : ModelID {
  kHouseGarden = 0,
  kBaxall = 1,
  kChurch = 2,
  kFireTrainingCenter = 3,
  kWoodIsland = 4,
  kRochesterWarehouse = 5,
  kGardenOfEden = 6,
  kBuffaloConstructionSite = 7,
  kConstructionSite = 8,
  kTotNumTrainingModel = 9
};

enum TestingModel3DId : ModelID {
  kEssexCastle = 9,
  kFraserGunneryRange = 10,
  kHartleyMansion = 11,
  kTinHauTemple = 12,
  kXcellAerialHq = 13,
  kIrchel = 14,
  kHofdiHouse = 15,
  kTotNumTestingModel = 7
};

inline std::string objFileFromId(const ModelID model_id) {
  std::string model_name;
  switch (model_id) {
    case TrainingModel3DId::kHouseGarden:
      model_name = "house_garden";
      break;
    case TrainingModel3DId::kBaxall:
      model_name = "baxall";
      break;
    case TrainingModel3DId::kChurch:
      model_name = "church";
      break;
    case TrainingModel3DId::kFireTrainingCenter:
      model_name = "fire_training_center";
      break;
    case TrainingModel3DId::kWoodIsland:
      model_name = "wood_island";
      break;
    case TrainingModel3DId::kRochesterWarehouse:
      model_name = "rochester_warehouse";
      break;
    case TrainingModel3DId::kGardenOfEden:
      model_name = "garden_of_eden";
      break;
    case TrainingModel3DId::kBuffaloConstructionSite:
      model_name = "buffalo_construction_site";
      break;
    case TrainingModel3DId::kConstructionSite:
      model_name = "construction_site";
      break;
    case TestingModel3DId::kEssexCastle:
      model_name = "essex_castle";
      break;
    case TestingModel3DId::kFraserGunneryRange:
      model_name = "fraser_gunnery_range";
      break;
    case TestingModel3DId::kHartleyMansion:
      model_name = "hartley_mansion";
      break;
    case TestingModel3DId::kTinHauTemple:
      model_name = "tin_hau_temple";
      break;
    case TestingModel3DId::kXcellAerialHq:
      model_name = "xcell_aerial_hq";
      break;
    case TestingModel3DId::kIrchel:
      model_name = "irchel";
      break;
    case TestingModel3DId::kHofdiHouse:
      model_name = "hofdi_house";
      break;
    default:
      model_name = "unknown";
      break;
  }
  return model_name;
}

inline ModelID objIdFromName(const std::string model_name) {
  ModelID model_id;

  if (model_name == "house_garden")
    model_id = TrainingModel3DId::kHouseGarden;
  else if (model_name == "baxall")
    model_id = TrainingModel3DId::kBaxall;
  else if (model_name == "church")
    model_id = TrainingModel3DId::kChurch;
  else if (model_name == "fire_training_center")
    model_id = TrainingModel3DId::kFireTrainingCenter;
  else if (model_name == "wood_island")
    model_id = TrainingModel3DId::kWoodIsland;
  else if (model_name == "rochester_warehouse")
    model_id = TrainingModel3DId::kRochesterWarehouse;
  else if (model_name == "garden_of_eden")
    model_id = TrainingModel3DId::kGardenOfEden;
  else if (model_name == "buffalo_construction_site")
    model_id = TrainingModel3DId::kBuffaloConstructionSite;
  else if (model_name == "construction_site")
    model_id = TrainingModel3DId::kConstructionSite;
  else if (model_name == "essex_castle")
    model_id = TestingModel3DId::kEssexCastle;
  else if (model_name == "fraser_gunnery_range")
    model_id = TestingModel3DId::kFraserGunneryRange;
  else if (model_name == "hartley_mansion")
    model_id = TestingModel3DId::kHartleyMansion;
  else if (model_name == "tin_hau_temple")
    model_id = TestingModel3DId::kTinHauTemple;
  else if (model_name == "xcell_aerial_hq")
    model_id = TestingModel3DId::kXcellAerialHq;
  else if (model_name == "irchel")
    model_id = TestingModel3DId::kIrchel;
  else if (model_name == "hofdi_house")
    model_id = TestingModel3DId::kHofdiHouse;
  else
    throw std::invalid_argument("Unknown model");

  return model_id;
}

}  // end namespace ml

#endif  // __MODEL_DEFS_HPP__
