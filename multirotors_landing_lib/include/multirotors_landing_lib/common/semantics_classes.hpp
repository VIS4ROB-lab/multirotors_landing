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

#ifndef __SEMANTICS_CLASSES_HPP__
#define __SEMANTICS_CLASSES_HPP__

#include <cmath>
#include <opencv2/core/core.hpp>

namespace ml {

struct Color {
  Color() : r(0), g(0), b(0) {}
  Color(const int _r, const int _g, const int _b) : r(_r), g(_g), b(_b) {}
  int r, g, b;
};

inline bool operator==(const Color &c1, const Color &c2) {
  return c1.r == c2.r && c1.g == c2.g && c1.b == c2.b;
}

enum SemClasses : int {
  kUnknown = -1,
  kPavement = 0,
  kTerrain = 1,
  kWater = 2,
  kSky = 3,
  kBuilding = 4,
  kVegetation = 5,
  kPerson = 6,
  kRider = 7,
  kVehicle = 8,
  kOthers = 9
};

// The colors and their conversions RGB <-> Gray are defined here:
// https://github.com/VIS4ROB-lab/v4rl_aerial_semantic_dataset/blob/main/blender_scripts/convert_semantic_image.py

inline Color fromClassToRgbColor(const int sem_class) {
  switch (sem_class) {
    case kUnknown:
      return Color(0, 0, 0);  // Gray intensity: 0
    case kPavement:
      return Color(81, 0, 81);  // Gray intensity: 10
    case kTerrain:
      return Color(152, 251, 152);  // Gray intensity: 30
    case kWater:
      return Color(150, 170, 250);  // Gray intensity: 60
    case kSky:
      return Color(70, 130, 180);  // Gray intensity: 70
    case kBuilding:
      return Color(70, 70, 70);  // Gray intensity: 20
    case kVegetation:
      return Color(107, 142, 35);  // Gray intensity: 40
    case kPerson:
      return Color(220, 20, 60);  // Gray intensity: 80
    case kRider:
      return Color(255, 0, 0);  // Gray intensity: 90
    case kVehicle:
      return Color(0, 0, 142);  // Gray intensity: 50
    case kOthers:
      return Color(250, 170, 30);  // Gray intensity: 250
    default:
      return Color(0, 0, 0);
  }
}

inline int fromClassToGrayIntesity(const int sem_class) {
  switch (sem_class) {
    case kUnknown:
      return 0;
    case kPavement:
      return 10;
    case kTerrain:
      return 30;
    case kWater:
      return 60;
    case kSky:
      return 70;
    case kBuilding:
      return 20;
    case kVegetation:
      return 40;
    case kPerson:
      return 80;
    case kRider:
      return 90;
    case kVehicle:
      return 50;
    case kOthers:
      return 250;
    default:
      return 0;
  }
}

inline SemClasses fromGrayIntensityToClass(const int intensity) {
  // Add multiple cases to avoid problems with float -> int coversion in vulkan
  if (intensity <= 1)
    return kUnknown;
  else if (intensity >= 9 && intensity <= 11)
    return kPavement;
  else if (intensity >= 29 && intensity <= 31)
    return kTerrain;
  else if (intensity >= 59 && intensity <= 61)
    return kWater;
  else if (intensity >= 69 && intensity <= 71)
    return kSky;
  else if (intensity >= 19 && intensity <= 21)
    return kBuilding;
  else if (intensity >= 39 && intensity <= 41)
    return kVegetation;
  else if (intensity >= 79 && intensity <= 81)
    return kPerson;
  else if (intensity >= 89 && intensity <= 91)
    return kRider;
  else if (intensity >= 49 && intensity <= 51)
    return kVehicle;
  else if (intensity >= 249 && intensity <= 251)
    return kOthers;
  else
    return kUnknown;
}

inline SemClasses fromRgbColorToClass(const Color &color) {
  if (color == Color(0, 0, 0))
    return kUnknown;
  else if (color == Color(81, 0, 81))
    return kPavement;
  else if (color == Color(152, 251, 152))
    return kTerrain;
  else if (color == Color(150, 170, 250))
    return kWater;
  else if (color == Color(70, 130, 180))
    return kSky;
  else if (color == Color(70, 70, 70))
    return kBuilding;
  else if (color == Color(107, 142, 35))
    return kVegetation;
  else if (color == Color(220, 20, 60))
    return kPerson;
  else if (color == Color(255, 0, 0))
    return kRider;
  else if (color == Color(0, 0, 142))
    return kVehicle;
  else if (color == Color(250, 170, 30))
    return kOthers;
  else
    return kUnknown;
}

inline std::string stringFromClassId(const int id) {
  switch (id) {
    case kUnknown:
      return "Unknown";
    case kPavement:
      return "Pavement";
    case kTerrain:
      return "Terrain";
    case kWater:
      return "Water";
    case kSky:
      return "Sky";
    case kBuilding:
      return "Building";
    case kVegetation:
      return "Vegetation";
    case kPerson:
      return "Person";
    case kRider:
      return "Rider";
    case kVehicle:
      return "Vehicle";
    case kOthers:
      return "Others";
    default:
      return "Unknown";
  }
}

inline cv::Mat getColorSemantic(const cv::Mat &grayscale_semantics) {
  cv::Mat color_semantics(grayscale_semantics.size(), CV_8UC3);  // B-G-R
  for (int u = 0; u < grayscale_semantics.cols; ++u) {
    for (int v = 0; v < grayscale_semantics.rows; ++v) {
      // Get the color from the semantic class
      const int pixel = (int)grayscale_semantics.at<uchar>(v, u);
      const SemClasses sem_class = fromGrayIntensityToClass(pixel);
      Color color = fromClassToRgbColor(sem_class);
      //
      color_semantics.at<cv::Vec3b>(v, u)[0] = static_cast<uchar>(color.b);
      color_semantics.at<cv::Vec3b>(v, u)[1] = static_cast<uchar>(color.g);
      color_semantics.at<cv::Vec3b>(v, u)[2] = static_cast<uchar>(color.r);
    }
  }
  return color_semantics;
}

}  // end namespace ml

#endif  // __SEMANTICS_CLASSES_HPP__
