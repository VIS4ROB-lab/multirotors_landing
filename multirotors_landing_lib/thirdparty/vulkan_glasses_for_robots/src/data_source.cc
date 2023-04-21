
#include <glog/logging.h>
#include <vulkan_glasses_for_robots/data_source.h>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/tokenizer.hpp>
#include <fstream>
#include <iomanip>
#include <iostream>

DataSource* DataSource::Create(const std::string& project_folder,
                               const std::string& camera_id_) {
  if (boost::filesystem::exists(project_folder)) {
    DataSource* data_source = new DataSource();
    data_source->project_folder_path_ = project_folder;
    data_source->camera_id_ = camera_id_;
    if (data_source->load_poses())
      return data_source;
    else
      delete data_source;
  }
  LOG(ERROR) << "could not load the datasource:" << project_folder;

  return nullptr;
}

void DataSource::getPositions(cv::Mat& positions) {
  positions.create(data_.size(), 3, CV_32F);

  for (size_t i = 0; i < data_.size(); i++) {
    const auto pos = data_[i].T_WC.block<3, 1>(0, 3).cast<float>();
    positions.at<float>(i, 0) = pos(0);
    positions.at<float>(i, 1) = pos(1);
    positions.at<float>(i, 2) = pos(2);
  }
}

bool DataSource::load_poses() {
  boost::filesystem::path poses_file =
      project_folder_path_ / "output/1_InertialPose/pose_data.csv";

  std::ifstream file(poses_file.c_str(), std::ifstream::in);

  if (!file.is_open()) {
    LOG(ERROR) << "cannot open " << poses_file.c_str() << "#";
    return false;
  }

  std::string line;
  std::getline(file, line);  // drop first line
  int skiper = 0;
  int factor = visim_project_.sampling_factor;
  while (std::getline(file, line)) {
    if (skiper++ % factor != 0) {
      continue;
    }

    std::vector<std::string> strs;
    boost::split(strs, line, boost::is_any_of(","));

    if (strs.size() != 8) {
      LOG(ERROR) << "format problem line #" << line << "#";
      return false;
    }
    DataEntry curr_entry;
    curr_entry.timestamp = boost::lexical_cast<uint64_t>(strs[0]);

    Eigen::Vector3d p_WS(boost::lexical_cast<double>(strs[1]),
                         boost::lexical_cast<double>(strs[2]),
                         boost::lexical_cast<double>(strs[3]));

    Eigen::Quaterniond q_WS;
    q_WS.x() = boost::lexical_cast<double>(strs[4]);
    q_WS.y() = boost::lexical_cast<double>(strs[5]);
    q_WS.z() = boost::lexical_cast<double>(strs[6]);
    q_WS.w() = boost::lexical_cast<double>(strs[7]);

    Eigen::Matrix4d T_WS(Eigen::Matrix4d::Identity());
    T_WS.block<3, 3>(0, 0) = q_WS.toRotationMatrix();
    T_WS.block<3, 1>(0, 3) = p_WS;

    curr_entry.T_WC = T_WS * visim_project_.T_SC;
    curr_entry.sequence = data_.size() * factor + 1;

    data_.push_back(curr_entry);
    timestamp_index_[curr_entry.timestamp] = data_.size() - 1;  // last element
    // LOG(INFO) << "ts #" << curr_entry.timestamp << "###";
  }
  if (data_.size() == 0) {
    LOG(ERROR) << "no poses found in " << poses_file.c_str();
    return false;
  }

  return true;
}

bool DataSource::searchId(uint64_t image_timestamp, size_t& index) {
  if (timestamp_index_.count(
          image_timestamp -
          visim_project_.imu_image_delay)) {  // the minus 1 is because the
                                              // images has delay of 1nanosec
                                              // with respect to the imu
    index = timestamp_index_[image_timestamp - visim_project_.imu_image_delay];
    return true;
  }

  return false;
}

bool DataSource::searchId(std::string string_image_timestamp, size_t& index) {
  try {
    uint64_t ts = boost::lexical_cast<uint64_t>(string_image_timestamp);
    return searchId(ts, index);
  } catch (const boost::bad_lexical_cast& e) {
    LOG(ERROR) << e.what();
  }
  return false;
}
