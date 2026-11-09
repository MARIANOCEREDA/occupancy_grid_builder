#pragma once

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <string>

namespace pcl_labeler
{
class ExtrinsicsParser
{
 public:
  ExtrinsicsParser(const std::string& extrinsics_file_path);
  Eigen::Matrix4f get_matrix() const;
  Eigen::Matrix4f get_inverse_matrix() const;
  Eigen::Matrix3f get_rotation_matrix() const;
  Eigen::Vector3f get_translation_vector() const;

 private:
  Eigen::Matrix4f parse_extrinsics_file(const std::string& file_path);

  Eigen::Matrix4f extrinsics_;
};
}  // namespace pcl_labeler