#ifndef MESH_DEPTH_EVALUATOR_DATA_SOURCE_H_
#define MESH_DEPTH_EVALUATOR_DATA_SOURCE_H_

#include <boost/filesystem.hpp>
#include <map>
//#include <kindr/minimal/quat-transformation.h>
#include <Eigen/Dense>
#include <opencv2/core.hpp>

class VisimProject {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double f;
  float cx, cy;
  int w, h, sampling_factor, imu_image_delay;
  Eigen::Matrix4d T_SC;  //kindr::minimal::QuatTransformation T_SC;
  VisimProject() {
    f = 455;
    cx = 376.5;
    cy = 240.5;
    w = 752;
    h = 480;
    imu_image_delay = 1;  // nanosec

    sampling_factor = 10;
    T_SC << 0, 0, 1, 0.015, -1, 0, 0, 0.055, 0, -1, 0, 0.0065, 0, 0, 0, 1;
  }
  // todo load from json project
};

struct DataEntry {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  uint64_t timestamp;
  Eigen::Matrix4d T_WC;
  uint sequence;
};

class DataSource {
  public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  DataSource() {}

  static DataSource* Create(const std::string& project_folder,
                            const std::string& camera_id_);

  void getPositions(cv::Mat& positions);

  bool getDepthImage(size_t index, cv::Mat& image);
  bool getRGBDImage(size_t index, cv::Mat& image_r, cv::Mat& image_g,
                    cv::Mat& image_b, cv::Mat& image_z);

  bool load_poses();

  bool searchId(std::string string_image_timestamp, size_t& index);
  bool searchId(uint64_t image_timestamp, size_t& index);

  const VisimProject& getProject() {
    return visim_project_;
  }

  const DataEntry& at(size_t i) {
    return data_[i];
  }

  size_t size() {
    return data_.size();
  }

  private:
  boost::filesystem::path project_folder_path_;
  std::string camera_id_;
  VisimProject visim_project_;
  std::vector<DataEntry, Eigen::aligned_allocator<DataEntry>> data_;
  std::map<uint64_t, size_t> timestamp_index_;
};

#endif
