#ifndef POSE_TRANSFORMER_
#define POSE_TRANSFORMER_

#include <Eigen/Eigen>
#include <Eigen/src/Core/Matrix.h>
#include <iostream>

namespace poseTransformer {

class poseTransformer {
public:
  // rotation transform
  Eigen::Matrix3f eulerToR(Eigen::Vector3f euler);
  Eigen::Vector3f RToEuler(Eigen::Matrix3f r);

  Eigen::Matrix3f quatToR(Eigen::Vector4f quat);
  Eigen::Vector4f RToQuat(Eigen::Matrix3f r);

  Eigen::Vector4f eulerToQuat(Eigen::Vector3f euler);
  Eigen::Vector3f quatToEuler(Eigen::Vector4f quat);

  // pose transform
  Eigen::Matrix4f pose3dEulerToM(Eigen::VectorXf trans_euler);
  Eigen::VectorXf pose3dMToEuler(Eigen::Matrix4f m);

  Eigen::Matrix4f pose3dQuatToM(Eigen::VectorXf trans_quat);
  Eigen::VectorXf pose3dMToQuat(Eigen::Matrix4f m);

  Eigen::VectorXf pose3dEulerToQuat(Eigen::VectorXf trans_euler);
  Eigen::VectorXf pose3dQuatToEuler(Eigen::VectorXf trans_quat);

  // setters
  bool resetFromEuler(Eigen::Vector3f euler);
  bool resetFromQuat(Eigen::Vector4f quat);
  bool resetFromR(Eigen::Matrix3f r);
  bool resetFromPose3dEuler(Eigen::VectorXf trans_euler);
  bool resetFromPose3dQuat(Eigen::VectorXf trans_quat);
  bool resetFromPose3dM(Eigen::Matrix4f m);

  // getters
  const Eigen::Vector3f getEuler() const;
  const Eigen::Vector4f getQuat() const;
  const Eigen::Matrix3f getR() const;
  const Eigen::VectorXf getPose3dEuler() const;
  const Eigen::VectorXf getPose3dQuat() const;
  const Eigen::Matrix4f getPose3dM() const;

  // reset
  bool reset();

private:
  Eigen::Matrix4f m_ = Eigen::Matrix4f::Identity();
};
}

#endif