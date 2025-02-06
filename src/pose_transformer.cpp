#include "pose_transformer.hpp"

namespace poseTransformer {

Eigen::Matrix3f poseTransformer::eulerToR(Eigen::Vector3f euler){
  resetFromEuler(euler);
  return getR();
}
Eigen::Vector3f poseTransformer::RToEuler(Eigen::Matrix3f r) {
  resetFromR(r);
  return getEuler();
}

Eigen::Matrix3f poseTransformer::quatToR(Eigen::Vector4f quat) {
  resetFromQuat(quat);
  return getR();
}
Eigen::Vector4f poseTransformer::RToQuat(Eigen::Matrix3f r) {
  resetFromR(r);
  return getQuat();
}

Eigen::Vector4f poseTransformer::eulerToQuat(Eigen::Vector3f euler) {
  resetFromEuler(euler);
  return getQuat();
}
Eigen::Vector3f poseTransformer::quatToEuler(Eigen::Vector4f quat) {
  resetFromQuat(quat);
  return getEuler();
}

// pose transform
Eigen::Matrix4f poseTransformer::pose3dEulerToM(Eigen::VectorXf trans_euler) {
  resetFromPose3dEuler(trans_euler);
  return getPose3dM();
}
Eigen::VectorXf poseTransformer::pose3dMToEuler(Eigen::Matrix4f m) {
  resetFromPose3dM(m);
  return getPose3dEuler();
}

Eigen::Matrix4f poseTransformer::pose3dQuatToM(Eigen::VectorXf trans_quat) {
  resetFromPose3dQuat(trans_quat);
  return getPose3dM();
}
Eigen::VectorXf poseTransformer::pose3dMToQuat(Eigen::Matrix4f m) {
  resetFromPose3dM(m);
  return getPose3dQuat();
}

Eigen::VectorXf
poseTransformer::pose3dEulerToQuat(Eigen::VectorXf trans_euler) {
  resetFromPose3dEuler(trans_euler);
  return getPose3dQuat();
}
Eigen::VectorXf poseTransformer::pose3dQuatToEuler(Eigen::VectorXf trans_quat) {
  resetFromPose3dQuat(trans_quat);
  return getPose3dEuler();
}

// setters
bool poseTransformer::resetFromEuler(Eigen::Vector3f euler) {
  reset();

  Eigen::AngleAxisf aa;
  aa = Eigen::AngleAxisf(euler(0), Eigen::Vector3f::UnitZ()) *
       Eigen::AngleAxisf(euler(1), Eigen::Vector3f::UnitY()) *
       Eigen::AngleAxisf(euler(2), Eigen::Vector3f::UnitX());
  m_.block(0, 0, 3, 3) = aa.toRotationMatrix();

  return true;
}
bool poseTransformer::resetFromQuat(Eigen::Vector4f quat) {
  reset();

  Eigen::AngleAxisf aa;
  aa = Eigen::Quaternionf(quat);
  m_.block(0, 0, 3, 3) = aa.toRotationMatrix();

  return true;
}
bool poseTransformer::resetFromR(Eigen::Matrix3f r) {
  reset();

  m_.block(0, 0, 3, 3) = r;

  return true;
}
bool poseTransformer::resetFromPose3dEuler(Eigen::VectorXf trans_euler) {
  resetFromEuler(
      Eigen::Vector3f(trans_euler(3), trans_euler(4), trans_euler(5)));
  m_.block(0, 3, 3, 1) << trans_euler(0), trans_euler(1), trans_euler(2);

  return true;
}
bool poseTransformer::resetFromPose3dQuat(Eigen::VectorXf trans_quat) {
  resetFromQuat(Eigen::Vector4f(trans_quat(3), trans_quat(4), trans_quat(5),
                                trans_quat(6)));
  m_.block(0, 3, 3, 1) << trans_quat(0), trans_quat(1), trans_quat(2);
  return true;
}
bool poseTransformer::resetFromPose3dM(Eigen::Matrix4f m) {
  m_ = m;
  return true;
}

// getters
const Eigen::Vector3f poseTransformer::getEuler() const {
  Eigen::Vector3f euler;
  Eigen::Matrix3f R = m_.block(0, 0, 3, 3);
  //   euler = m_.block(0, 0, 3, 3).eulerAngles(0, 1, 2);
  euler = R.eulerAngles(2, 1, 0);
  return euler;
}
const Eigen::Vector4f poseTransformer::getQuat() const {
  Eigen::AngleAxisf aa;
  Eigen::Quaternionf eig_quat;
  Eigen::Matrix3f R = m_.block(0, 0, 3, 3);
  aa = R;
  eig_quat = aa;

  Eigen::Vector4f quat;
  quat << eig_quat.coeffs();

  return quat;
}
const Eigen::Matrix3f poseTransformer::getR() const {
  return m_.block(0, 0, 3, 3);
}
const Eigen::VectorXf poseTransformer::getPose3dEuler() const {
  Eigen::Vector3f euler = getEuler();
  Eigen::VectorXf trans_euler(6);
  trans_euler << m_(0, 3), m_(1, 3), m_(2, 3), euler(0), euler(1), euler(2);

  return trans_euler;
}
const Eigen::VectorXf poseTransformer::getPose3dQuat() const {
  Eigen::Vector4f quat = getQuat();
  Eigen::VectorXf trans_quat(7);
  trans_quat << m_(0, 3), m_(1, 3), m_(2, 3), quat(0), quat(1), quat(2),
      quat(3);

  return trans_quat;
}
const Eigen::Matrix4f poseTransformer::getPose3dM() const {
  return m_;
}

// reset
bool poseTransformer::reset() {
  m_ = Eigen::Matrix4f::Identity();
  return true;
}
}