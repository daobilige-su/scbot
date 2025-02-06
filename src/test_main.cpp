#include "pose_transformer.hpp"
#include <iostream>

int main() {
  Eigen::Vector3f euler;
  Eigen::Matrix3f R;
  Eigen::Matrix3f R_GT;
  poseTransformer::poseTransformer transformer;

  euler << 0, 0, 0;
  R_GT = Eigen::Matrix3f::Identity();
  transformer.resetFromEuler(euler);
  R = transformer.getR();
  std::cout<< R << std::endl;
}