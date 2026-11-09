#include <fstream>
#include <pcl_labeler/extrinsics_parser.hpp>
#include <stdexcept>

namespace pcl_labeler
{
ExtrinsicsParser::ExtrinsicsParser(const std::string& extrinsics_file_path)
{
  extrinsics_ = parse_extrinsics_file(extrinsics_file_path);
}

Eigen::Matrix4f ExtrinsicsParser::parse_extrinsics_file(const std::string& file_path)
{
  std::ifstream file(file_path);
  if (!file.is_open())
  {
    throw std::runtime_error("Could not open extrinsics file: " + file_path);
  }

  nlohmann::json json_data;
  file >> json_data;

  // Support a nested 4x4 array under an "extrinsics" key or directly at the root
  const nlohmann::json& matrix_data =
      json_data.contains("extrinsics") ? json_data["extrinsics"] : json_data;

  if (!matrix_data.is_array() || matrix_data.size() != 4)
  {
    throw std::runtime_error("Extrinsics matrix must be a 4x4 array");
  }

  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  for (int row = 0; row < 4; ++row)
  {
    if (!matrix_data[row].is_array() || matrix_data[row].size() != 4)
    {
      throw std::runtime_error("Each row of the extrinsics matrix must have 4 elements");
    }
    for (int col = 0; col < 4; ++col)
    {
      matrix(row, col) = matrix_data[row][col].get<float>();
    }
  }

  return matrix;
}

Eigen::Matrix4f ExtrinsicsParser::get_matrix() const
{
  return extrinsics_;
}

Eigen::Matrix4f ExtrinsicsParser::get_inverse_matrix() const
{
  return extrinsics_.inverse();
}

Eigen::Matrix3f ExtrinsicsParser::get_rotation_matrix() const
{
  return extrinsics_.block<3, 3>(0, 0);
}

Eigen::Vector3f ExtrinsicsParser::get_translation_vector() const
{
  return extrinsics_.block<3, 1>(0, 3);
}
}  // namespace pcl_labeler