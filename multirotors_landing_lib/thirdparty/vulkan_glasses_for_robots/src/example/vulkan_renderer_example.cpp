/**
 * @brief Example script to show how to use the renderer given an *.obj model
 *        and its textures. To get the path to the model, we assume that there
 *        exits the variable MODELS_FOLDER and the model and textures are at :
 *        MODELS_FOLDER/model_name/model_name.obj, 
 *        MODELS_FOLDER/model_name/model_name_rgbs.png
 */

#include <unistd.h>

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

#include "vulkan_glasses_for_robots/vulkan_renderer.h"

void buildOpenglProjectionFromIntrinsics(glm::mat4 &matPerspective,
                                         glm::mat4 &matProjection,
                                         int img_width, int img_height,
                                         float alpha, float beta, float skew,
                                         float u0, float v0, float near,
                                         float far) {
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
  ndc[3][3] = 1.f;

  // construct a projection matrix, this is identical to the projection matrix
  // computed for the intrinsic,
  // except an additional row is inserted to map the z-coordinate to OpenGL.
  // CMatrix4<T> matProjection(0);    // the 3rd column is inverted to make the
  // camera look towards +Z (instead of -Z in opengl)
  matProjection = glm::mat4(0);
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
  matPerspective = ndc * matProjection;
  matPerspective[1][1] *=
      -1;  // was originally designed for OpenGL, where the Y coordinate of the
           // clip coordinates is inverted in relation Vulkan.
}

glm::mat4 computeMVP(const Eigen::Matrix4d &T_WC,
                     const glm::mat4 &perspective) {
  const auto T_CW_cv_eigen = T_WC.inverse();
  glm::mat4 T_CW_cv_glm;
  glm::mat4 conversion_gl_cv =
      glm::mat4(1, 0, 0, 0, 0, -1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1);

  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      T_CW_cv_glm[j][i] = static_cast<float>(T_CW_cv_eigen(i, j));
    }
  }
  return perspective * conversion_gl_cv * T_CW_cv_glm;
}

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  LOG(INFO) << "Starting vulkan renderer example";
  if (getenv("MODELS_FOLDER") != nullptr) {
    // Here we assume that the model is inside: MODELS_FOLDER/model_name/model_name.obj
    const std::string model_name = "house_garden";
    const std::string full_path =
        getenv("MODELS_FOLDER") + std::string("/") + model_name;
    LOG(INFO) << "Loading model at: " << full_path;

    // Assuming textures are: full_path/model_name.rgbs
    const std::string mesh_obj_file(full_path + "/" + model_name + ".obj");
    const std::string texture_file(full_path + "/" + model_name + "_rgbs.png");

    // Actual renderer
    const std::string shader =
        get_current_dir_name() + std::string("/../shaders");
    const int width = 752, height = 480;
    const float near = 0.05f, far = 100.f;
    vrglasses_for_robots::VulkanRenderer renderer(width, height, near, far,
                                                  shader);
    LOG(INFO) << "Renderer initialized";

    const float f = 455.f, cx = 376.5f, cy = 240.5f;
    glm::mat4 perspective, empty;
    buildOpenglProjectionFromIntrinsics(perspective, empty, width, height, f, f,
                                        0, cx, cy, near, far);

    // Load model
    renderer.clearModels();
    renderer.loadMesh(mesh_obj_file, texture_file);
    renderer.noFileScene();
    LOG(INFO) << "Model loaded";

    // Render images
    Eigen::Matrix4d T_WB(Eigen::Matrix4d::Identity());  // Robot pose
    T_WB.block<3, 1>(0, 3) = Eigen::Vector3d(0, 0, 30);

    Eigen::Matrix4d T_BC(Eigen::Matrix4d::Identity());  // Calibration matrix
    T_BC.block<3, 1>(0, 3) = Eigen::Vector3d(0.015, 0.055, 0.0065);
    // This is to have a down-looking camera
    Eigen::Matrix<double, 3, 3> R_pitch(
        Eigen::AngleAxisd(-M_PI / 2, Eigen::Vector3d::UnitX()));
    Eigen::Matrix<double, 3, 3> R_SC0(Eigen::Matrix<double, 3, 3>::Zero());
    R_SC0(0, 2) = 1.0;
    R_SC0(1, 0) = -1.0;
    R_SC0(2, 1) = -1.0;
    T_BC.block<3, 3>(0, 0) = R_SC0 * R_pitch;

    glm::mat4 mvp = computeMVP(T_WB * T_BC, perspective);
    renderer.setCamera(mvp);

    // Initialize the output
    cv::Mat rgb_img, sem_img, depth_img;
    rgb_img.create(height, width, CV_8UC3);
    sem_img.create(height, width, CV_8UC1);
    depth_img.create(height, width, CV_32FC1);

    // Actual output
    cv::Mat rgbs_map;
    renderer.renderMesh(depth_img, rgbs_map);

    // Get the RGB and the Semantic images
    cv::Mat out[] = {rgb_img, sem_img};
    int from_to[] = {0, 0, 1, 1, 2, 2, 3, 3};
    cv::mixChannels(&rgbs_map, 1, out, 2, from_to, 4);

    cv::imshow("RGB", rgb_img);
    cv::imshow("Semantics", sem_img);
    cv::imshow("Depth", depth_img / 255);
    cv::waitKey();

  } else {
    LOG(ERROR) << "Environment variable 'MODELS_FOLDER' not declared - abort";
    return EXIT_FAILURE;
  }

  LOG(WARNING) << "Closing!";
  return EXIT_SUCCESS;
}
