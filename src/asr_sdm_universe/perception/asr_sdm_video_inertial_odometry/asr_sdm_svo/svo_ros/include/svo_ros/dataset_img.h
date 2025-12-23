#ifndef DATASET_IMG_H_
#define DATASET_IMG_H_

namespace FileType
{

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <string>

using ::Eigen::Quaterniond;
using ::Eigen::Vector3d;

/// Specification of Dataset which contains: Camera Image, Gyroscope Rotation
class DatasetImg
{
public:
  DatasetImg() {}
  virtual ~DatasetImg() {}

  double timestamp_;
  std::string image_name_;

  friend std::ostream & operator<<(std::ostream & out, const DatasetImg & pair);
  friend std::istream & operator>>(std::istream & in, DatasetImg & pair);
};

std::ostream & operator<<(std::ostream & out, const DatasetImg & gt)
{
  out << gt.timestamp_ << " " << gt.image_name_ << " " << std::endl;
  return out;
}

std::istream & operator>>(std::istream & in, DatasetImg & gt)
{
  in >> gt.timestamp_;
  in >> gt.image_name_;
  return in;
}

}  // end namespace FileType

#endif /* DATASET_IMG_H */
