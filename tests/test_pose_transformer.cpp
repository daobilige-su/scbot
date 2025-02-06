#include "pose_transformer.hpp"
#include <Eigen/src/Core/EigenBase.h>
#include <gtest/gtest.h>
#include <cmath>
#include <iomanip>

bool debug_print_on = false;

template <typename T>
void setVectorSmallNumToZero(T &vec, float thr = 0.000001) {
  for (int i = 0; i < vec.size(); ++i) {
    if (abs(vec(i)) < thr) {
      vec(i) = 0;
    }
  }
}

template <typename T>
void setMatrixSmallNumToZero(T &m, float thr = 0.000001) {
  for (int i = 0; i < m.rows(); ++i) {
    for (int j = 0; j < m.cols(); ++j) {
      if (abs(m(i, j)) < thr) {
        m(i, j) = 0;
      }
    }
  }
}

TEST(PoseTransformer, EulerToR) {
  Eigen::Vector3f euler;
  Eigen::Matrix3f R, R_GT;
  poseTransformer::poseTransformer transformer;

  euler << 0, 0, 0;
  R_GT = Eigen::Matrix3f::Identity();
  transformer.resetFromEuler(euler);
  R = transformer.getR();
  EXPECT_EQ(R, R_GT);

  euler << (M_PI / 4.0), (M_PI / 4.0), 0;
  R_GT << 0.5, -0.7071, 0.5, 0.5, 0.7071, 0.5, -0.7071, 0, 0.7071;
  transformer.resetFromEuler(euler);
  R = transformer.getR();
  setMatrixSmallNumToZero(R);
  std::stringstream ss, ss_GT;
  ss.setf(std::ios::fixed, std::ios::floatfield);
  ss.precision(4);
  ss_GT.setf(std::ios::fixed, std::ios::floatfield);
  ss_GT.precision(4);
  ss << R(0, 0) << ", " << R(0, 1) << ", " << R(0, 2) << ", "
     << R(1, 0) << ", " << R(1, 1) << ", " << R(1, 2) << ", "
     << R(2, 0) << ", " << R(2, 1) << ", " << R(2, 2);
  ss_GT << R_GT(0, 0) << ", " << R_GT(0, 1) << ", " << R_GT(0, 2) << ", "
        << R_GT(1, 0) << ", " << R_GT(1, 1) << ", " << R_GT(1, 2) << ", "
        << R_GT(2, 0) << ", " << R_GT(2, 1) << ", " << R_GT(2, 2);
  if (debug_print_on){
    std::cout << ss.str() << std::endl;
    std::cout << ss_GT.str() << std::endl;
  }
  EXPECT_EQ(ss.str(), ss_GT.str());
}

TEST(PoseTransformer, RToEuler) {
  Eigen::Vector3f euler, euler_GT;
  Eigen::Matrix3f R;
  poseTransformer::poseTransformer transformer;

  euler_GT << (M_PI / 4.0), (M_PI / 4.0), 0;
  R << 0.5, -0.7071, 0.5, 0.5, 0.7071, 0.5, -0.7071, 0, 0.7071;
  transformer.resetFromR(R);
  euler = transformer.getEuler();
  setVectorSmallNumToZero(euler);
  std::stringstream ss, ss_GT;
  ss.setf(std::ios::fixed, std::ios::floatfield);
  ss.precision(4);
  ss_GT.setf(std::ios::fixed, std::ios::floatfield);
  ss_GT.precision(4);
  ss << euler(0) << ", " << euler(1) << ", " << euler(2);
  ss_GT << euler_GT(0) << ", " << euler_GT(1) << ", " << euler_GT(2);
  if (debug_print_on) {
    std::cout << ss.str() << std::endl;
    std::cout << ss_GT.str() << std::endl;
  }
  EXPECT_EQ(ss.str(), ss_GT.str());
}

TEST(PoseTransformer, QuatToR) {
  Eigen::Vector4f quat;
  Eigen::Matrix3f R, R_GT;
  poseTransformer::poseTransformer transformer;

  quat << -0.14644661, 0.35355339, 0.35355339, 0.85355339;
  R_GT << 0.5, -0.7071, 0.5, 0.5, 0.7071, 0.5, -0.7071, 0, 0.7071;
  transformer.resetFromQuat(quat);
  R = transformer.getR();
  setMatrixSmallNumToZero(R);
  std::stringstream ss, ss_GT;
  ss.setf(std::ios::fixed, std::ios::floatfield);
  ss.precision(4);
  ss_GT.setf(std::ios::fixed, std::ios::floatfield);
  ss_GT.precision(4);
  ss << R(0, 0) << ", " << R(0, 1) << ", " << R(0, 2) << ", "
     << R(1, 0) << ", " << R(1, 1) << ", " << R(1, 2) << ", "
     << R(2, 0) << ", " << R(2, 1) << ", " << R(2, 2);
  ss_GT << R_GT(0, 0) << ", " << R_GT(0, 1) << ", " << R_GT(0, 2) << ", "
        << R_GT(1, 0) << ", " << R_GT(1, 1) << ", " << R_GT(1, 2) << ", "
        << R_GT(2, 0) << ", " << R_GT(2, 1) << ", " << R_GT(2, 2);
  if (debug_print_on){
    std::cout << ss.str() << std::endl;
    std::cout << ss_GT.str() << std::endl;
  }
  EXPECT_EQ(ss.str(), ss_GT.str());
}

TEST(PoseTransformer, RToQuat) {
  Eigen::Vector4f quat, quat_GT;
  Eigen::Matrix3f R;
  poseTransformer::poseTransformer transformer;

  quat_GT << -0.14644661, 0.35355339, 0.35355339, 0.85355339;
  R << 0.5, -0.7071, 0.5, 0.5, 0.7071, 0.5, -0.7071, 0, 0.7071;
  transformer.resetFromR(R);
  quat = transformer.getQuat();
  setVectorSmallNumToZero(quat);
  std::stringstream ss, ss_GT;
  ss.setf(std::ios::fixed, std::ios::floatfield);
  ss.precision(4);
  ss_GT.setf(std::ios::fixed, std::ios::floatfield);
  ss_GT.precision(4);
  ss << quat(0) << ", " << quat(1) << ", " << quat(2) << ", " << quat(3);
  ss_GT << quat_GT(0) << ", " << quat_GT(1) << ", " << quat_GT(2) << ", "
        << quat_GT(3);
  if (debug_print_on){
    std::cout << ss.str() << std::endl;
    std::cout << ss_GT.str() << std::endl;
  }
  EXPECT_EQ(ss.str(), ss_GT.str());
}

TEST(PoseTransformer, EulerToQuat) {
  Eigen::Vector3f euler;
  Eigen::Vector4f quat, quat_GT;
  poseTransformer::poseTransformer transformer;

  euler << (M_PI / 4.0), (M_PI / 4.0), 0;
  quat_GT << -0.14644661, 0.35355339, 0.35355339, 0.85355339;
  transformer.resetFromEuler(euler);
  quat = transformer.getQuat();
  setVectorSmallNumToZero(quat);
  std::stringstream ss, ss_GT;
  ss.setf(std::ios::fixed, std::ios::floatfield);
  ss.precision(4);
  ss_GT.setf(std::ios::fixed, std::ios::floatfield);
  ss_GT.precision(4);
  ss << quat(0) << ", " << quat(1) << ", " << quat(2) << ", " << quat(3);
  ss_GT << quat_GT(0) << ", " << quat_GT(1) << ", " << quat_GT(2) << ", "
        << quat_GT(3);
  if (debug_print_on) {
    std::cout << ss.str() << std::endl;
    std::cout << ss_GT.str() << std::endl;
  }
  EXPECT_EQ(ss.str(), ss_GT.str());
}

TEST(PoseTransformer, QuatToEuler) {
  Eigen::Vector3f euler, euler_GT;
  Eigen::Vector4f quat;
  poseTransformer::poseTransformer transformer;

  euler_GT << (M_PI / 4.0), (M_PI / 4.0), 0;
  quat << -0.14644661, 0.35355339, 0.35355339, 0.85355339;
  transformer.resetFromQuat(quat);
  euler = transformer.getEuler();
  setVectorSmallNumToZero(euler);
  std::stringstream ss, ss_GT;
  ss.setf(std::ios::fixed, std::ios::floatfield);
  ss.precision(4);
  ss_GT.setf(std::ios::fixed, std::ios::floatfield);
  ss_GT.precision(4);
  ss << euler(0) << ", " << euler(1) << ", " << euler(2);
  ss_GT << euler_GT(0) << ", " << euler_GT(1) << ", " << euler_GT(2);
  if (debug_print_on) {
    std::cout << ss.str() << std::endl;
    std::cout << ss_GT.str() << std::endl;
  }
  EXPECT_EQ(ss.str(), ss_GT.str());
}

TEST(PoseTransformer, Pose3DEulerToM) {
  Eigen::VectorXf trans_euler(6);
  Eigen::Matrix4f M, M_GT;
  poseTransformer::poseTransformer transformer;

  trans_euler << 1, 2, 3.5, (M_PI / 4.0), (M_PI / 4.0), 0;
  M_GT << 0.5, -0.7071, 0.5, 1, 0.5, 0.7071, 0.5, 2, -0.7071, 0, 0.7071, 3.5, 0, 0, 0, 1;
  transformer.resetFromPose3dEuler(trans_euler);
  M = transformer.getPose3dM();
  setMatrixSmallNumToZero(M);
  std::stringstream ss, ss_GT;
  ss.setf(std::ios::fixed, std::ios::floatfield);
  ss.precision(4);
  ss_GT.setf(std::ios::fixed, std::ios::floatfield);
  ss_GT.precision(4);
  ss << M(0, 0) << ", " << M(0, 1) << ", " << M(0, 2) << ", " << M(0, 3) << ", "
     << M(1, 0) << ", " << M(1, 1) << ", " << M(1, 2) << ", " << M(1, 3) << ", "
     << M(2, 0) << ", " << M(2, 1) << ", " << M(2, 2) << ", " << M(2, 3) << ", "
     << M(3, 0) << ", " << M(3, 1) << ", " << M(3, 2) << ", " << M(3, 3);
  ss_GT << M_GT(0, 0) << ", " << M_GT(0, 1) << ", " << M_GT(0, 2) << ", " << M_GT(0, 3) << ", "
        << M_GT(1, 0) << ", " << M_GT(1, 1) << ", " << M_GT(1, 2) << ", " << M_GT(1, 3) << ", "
        << M_GT(2, 0) << ", " << M_GT(2, 1) << ", " << M_GT(2, 2) << ", " << M_GT(2, 3) << ", "
        << M_GT(3, 0) << ", " << M_GT(3, 1) << ", " << M_GT(3, 2) << ", " << M_GT(3, 3);
  if (debug_print_on) {
    std::cout << ss.str() << std::endl;
    std::cout << ss_GT.str() << std::endl;
  }
  EXPECT_EQ(ss.str(), ss_GT.str());

  M = transformer.pose3dEulerToM(trans_euler);
  setMatrixSmallNumToZero(M);
  ss.str("");
  ss_GT.str("");
  ss << M(0, 0) << ", " << M(0, 1) << ", " << M(0, 2) << ", " << M(0, 3) << ", "
     << M(1, 0) << ", " << M(1, 1) << ", " << M(1, 2) << ", " << M(1, 3) << ", "
     << M(2, 0) << ", " << M(2, 1) << ", " << M(2, 2) << ", " << M(2, 3) << ", "
     << M(3, 0) << ", " << M(3, 1) << ", " << M(3, 2) << ", " << M(3, 3);
  ss_GT << M_GT(0, 0) << ", " << M_GT(0, 1) << ", " << M_GT(0, 2) << ", " << M_GT(0, 3) << ", "
        << M_GT(1, 0) << ", " << M_GT(1, 1) << ", " << M_GT(1, 2) << ", " << M_GT(1, 3) << ", "
        << M_GT(2, 0) << ", " << M_GT(2, 1) << ", " << M_GT(2, 2) << ", " << M_GT(2, 3) << ", "
        << M_GT(3, 0) << ", " << M_GT(3, 1) << ", " << M_GT(3, 2) << ", " << M_GT(3, 3);
  if (debug_print_on) {
    std::cout << ss.str() << std::endl;
    std::cout << ss_GT.str() << std::endl;
  }
  EXPECT_EQ(ss.str(), ss_GT.str());
}

TEST(PoseTransformer, Pose3DMToEuler) {
  Eigen::VectorXf trans_euler(6), trans_euler_GT(6);
  Eigen::Matrix4f M;
  poseTransformer::poseTransformer transformer;

  trans_euler_GT << 1, 2, 3.5, (M_PI / 4.0), (M_PI / 4.0), 0;
  M << 0.5, -0.7071, 0.5, 1, 0.5, 0.7071, 0.5, 2, -0.7071, 0, 0.7071, 3.5, 0,
      0, 0, 1;
  transformer.resetFromPose3dM(M);
  trans_euler = transformer.getPose3dEuler();
  setVectorSmallNumToZero(trans_euler);
  std::stringstream ss, ss_GT;
  ss.setf(std::ios::fixed, std::ios::floatfield);
  ss.precision(4);
  ss_GT.setf(std::ios::fixed, std::ios::floatfield);
  ss_GT.precision(4);
  ss << trans_euler(0) << ", " << trans_euler(1) << ", " << trans_euler(2) << ", "
     << trans_euler(3) << ", " << trans_euler(4) << ", " << trans_euler(5);
  ss_GT << trans_euler_GT(0) << ", " << trans_euler_GT(1) << ", " << trans_euler_GT(2) << ", "
        << trans_euler_GT(3) << ", " << trans_euler_GT(4) << ", " << trans_euler_GT(5);
  if (debug_print_on) {
    std::cout << ss.str() << std::endl;
    std::cout << ss_GT.str() << std::endl;
  }
  EXPECT_EQ(ss.str(), ss_GT.str());

  trans_euler = transformer.pose3dMToEuler(M);
  setVectorSmallNumToZero(trans_euler);
  ss.str("");
  ss_GT.str("");
  ss << trans_euler(0) << ", " << trans_euler(1) << ", " << trans_euler(2) << ", "
     << trans_euler(3) << ", " << trans_euler(4) << ", " << trans_euler(5);
  ss_GT << trans_euler_GT(0) << ", " << trans_euler_GT(1) << ", " << trans_euler_GT(2) << ", "
        << trans_euler_GT(3) << ", " << trans_euler_GT(4) << ", " << trans_euler_GT(5);
  if (debug_print_on) {
    std::cout << ss.str() << std::endl;
    std::cout << ss_GT.str() << std::endl;
  }
  EXPECT_EQ(ss.str(), ss_GT.str());
}

TEST(PoseTransformer, Pose3DQuatToM) {
  Eigen::VectorXf trans_quat(7);
  Eigen::Matrix4f M, M_GT;
  poseTransformer::poseTransformer transformer;

  trans_quat << 1, 2, 3.5, -0.14644661, 0.35355339, 0.35355339, 0.85355339;
  M_GT << 0.5, -0.7071, 0.5, 1, 0.5, 0.7071, 0.5, 2, -0.7071, 0, 0.7071, 3.5, 0,
      0, 0, 1;
  transformer.resetFromPose3dQuat(trans_quat);
  M = transformer.getPose3dM();
  setMatrixSmallNumToZero(M);
  std::stringstream ss, ss_GT;
  ss.setf(std::ios::fixed, std::ios::floatfield);
  ss.precision(4);
  ss_GT.setf(std::ios::fixed, std::ios::floatfield);
  ss_GT.precision(4);
  ss << M(0, 0) << ", " << M(0, 1) << ", " << M(0, 2) << ", " << M(0, 3) << ", "
     << M(1, 0) << ", " << M(1, 1) << ", " << M(1, 2) << ", " << M(1, 3) << ", "
     << M(2, 0) << ", " << M(2, 1) << ", " << M(2, 2) << ", " << M(2, 3) << ", "
     << M(3, 0) << ", " << M(3, 1) << ", " << M(3, 2) << ", " << M(3, 3);
  ss_GT << M_GT(0, 0) << ", " << M_GT(0, 1) << ", " << M_GT(0, 2) << ", "
        << M_GT(0, 3) << ", " << M_GT(1, 0) << ", " << M_GT(1, 1) << ", "
        << M_GT(1, 2) << ", " << M_GT(1, 3) << ", " << M_GT(2, 0) << ", "
        << M_GT(2, 1) << ", " << M_GT(2, 2) << ", " << M_GT(2, 3) << ", "
        << M_GT(3, 0) << ", " << M_GT(3, 1) << ", " << M_GT(3, 2) << ", "
        << M_GT(3, 3);
  if (debug_print_on) {
    std::cout << ss.str() << std::endl;
    std::cout << ss_GT.str() << std::endl;
  }
  EXPECT_EQ(ss.str(), ss_GT.str());

  M = transformer.pose3dQuatToM(trans_quat);
  setMatrixSmallNumToZero(M);
  ss.str("");
  ss_GT.str("");
  ss << M(0, 0) << ", " << M(0, 1) << ", " << M(0, 2) << ", " << M(0, 3) << ", "
     << M(1, 0) << ", " << M(1, 1) << ", " << M(1, 2) << ", " << M(1, 3) << ", "
     << M(2, 0) << ", " << M(2, 1) << ", " << M(2, 2) << ", " << M(2, 3) << ", "
     << M(3, 0) << ", " << M(3, 1) << ", " << M(3, 2) << ", " << M(3, 3);
  ss_GT << M_GT(0, 0) << ", " << M_GT(0, 1) << ", " << M_GT(0, 2) << ", "
        << M_GT(0, 3) << ", " << M_GT(1, 0) << ", " << M_GT(1, 1) << ", "
        << M_GT(1, 2) << ", " << M_GT(1, 3) << ", " << M_GT(2, 0) << ", "
        << M_GT(2, 1) << ", " << M_GT(2, 2) << ", " << M_GT(2, 3) << ", "
        << M_GT(3, 0) << ", " << M_GT(3, 1) << ", " << M_GT(3, 2) << ", "
        << M_GT(3, 3);
  if (debug_print_on) {
    std::cout << ss.str() << std::endl;
    std::cout << ss_GT.str() << std::endl;
  }
  EXPECT_EQ(ss.str(), ss_GT.str());
}

TEST(PoseTransformer, Pose3DMToQuat) {
  Eigen::VectorXf trans_quat(7), trans_quat_GT(7);
  Eigen::Matrix4f M;
  poseTransformer::poseTransformer transformer;

  trans_quat_GT << 1, 2, 3.5, -0.14644661, 0.35355339, 0.35355339, 0.85355339;
  M << 0.5, -0.7071, 0.5, 1, 0.5, 0.7071, 0.5, 2, -0.7071, 0, 0.7071, 3.5, 0,
      0, 0, 1;
  transformer.resetFromPose3dM(M);
  trans_quat = transformer.getPose3dQuat();
  setVectorSmallNumToZero(trans_quat);
  std::stringstream ss, ss_GT;
  ss.setf(std::ios::fixed, std::ios::floatfield);
  ss.precision(4);
  ss_GT.setf(std::ios::fixed, std::ios::floatfield);
  ss_GT.precision(4);
  ss << trans_quat(0) << ", " << trans_quat(1) << ", " << trans_quat(2) << ", "
     << trans_quat(3) << ", " << trans_quat(4) << ", " << trans_quat(5) << ", "
     << trans_quat(6);
  ss_GT << trans_quat_GT(0) << ", " << trans_quat_GT(1) << ", "
        << trans_quat_GT(2) << ", " << trans_quat_GT(3) << ", "
        << trans_quat_GT(4) << ", " << trans_quat_GT(5) << ", "
        << trans_quat_GT(6);
  if (debug_print_on) {
    std::cout << ss.str() << std::endl;
    std::cout << ss_GT.str() << std::endl;
  }
  EXPECT_EQ(ss.str(), ss_GT.str());

  trans_quat = transformer.pose3dMToQuat(M);
  setVectorSmallNumToZero(trans_quat);
  ss.str("");
  ss_GT.str("");
  ss << trans_quat(0) << ", " << trans_quat(1) << ", " << trans_quat(2) << ", "
     << trans_quat(3) << ", " << trans_quat(4) << ", " << trans_quat(5) << ", "
     << trans_quat(6);
  ss_GT << trans_quat_GT(0) << ", " << trans_quat_GT(1) << ", "
        << trans_quat_GT(2) << ", " << trans_quat_GT(3) << ", "
        << trans_quat_GT(4) << ", " << trans_quat_GT(5) << ", "
        << trans_quat_GT(6);
  if (debug_print_on) {
    std::cout << ss.str() << std::endl;
    std::cout << ss_GT.str() << std::endl;
  }
  EXPECT_EQ(ss.str(), ss_GT.str());
}
