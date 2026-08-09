#pragma once

#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include <string>

namespace mask_lidar_fusion
{
class ExtrinsicsParser
{
 public:
  ExtrinsicsParser(const std::string& extrinsics_file_path);
  Eigen::Matrix4f GetMatrix() const;
  Eigen::Matrix4f GetInverseMatrix() const;
  Eigen::Matrix3f GetRotationMatrix() const;
  Eigen::Vector3f GetTranslationVector() const;

 private:
  // Add methods to parse the extrinsics file and provide transformation data
  Eigen::Matrix4f ParseExtrinsicsFile(const std::string& file_path);

  Eigen::Matrix4f extrinsics_;
};
}  // namespace mask_lidar_fusion