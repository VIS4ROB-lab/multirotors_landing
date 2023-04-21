
#ifndef VRGLASSES4ROBOTS_VISIM_PROCESSOR_H__
#define VRGLASSES4ROBOTS_VISIM_PROCESSOR_H__

#include <glog/logging.h>
#include <vulkan_glasses_for_robots/data_source.h>

#include <boost/filesystem.hpp>
#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/hash.hpp>

class VisimProcessor {
  public:
  VisimProcessor() {}
  bool initialization(const std::string& visim_project_folder,
                      const std::string& camera_id,
                      const std::string& output_folder_path, int step);

  void run();
  void runHeadless();

  private:
  glm::mat4 computeMVP(DataEntry& entry);
  glm::mat4 perpective_;
  DataSource* visim_data_source_;

  void buildOpenglProjectionFromIntrinsics(
      glm::mat4& matPerspective, glm::mat4& matProjection,
      /*glm::mat4& matCVProjection,*/ int img_width, int img_height,
      float alpha, float beta, float skew, float u0, float v0, float near,
      float far);
};

#endif
