#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <torch/torch.h>
#include <cstring>

#include "gs/gs/camera.cuh"
#include "gs/gs/camera_utils.cuh"

TEST(CameraUtilsTest, IdentityTransform) {
  Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
  Eigen::Vector3f t = Eigen::Vector3f::Zero();
  Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f result = getWorld2View2Eigen(R, t);
  EXPECT_TRUE(expected.isApprox(result, 1e-5f));
}

TEST(CameraUtilsTest, TranslationAndScale) {
  Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
  Eigen::Vector3f t = Eigen::Vector3f::Zero();
  Eigen::Vector3f translate(1.f, 0.f, 0.f);
  float scale = 2.f;
  Eigen::Matrix4f result = getWorld2View2Eigen(R, t, translate, scale);
  Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
  expected(0, 3) = -2.f;
  EXPECT_TRUE(expected.isApprox(result, 1e-5f));
}

TEST(CameraUtilsTest, TorchIdentityTransform) {
  Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
  Eigen::Vector3f t = Eigen::Vector3f::Zero();
  Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
  torch::Tensor tensor = getWorld2View2(R, t);
  Eigen::Matrix4f result;
  std::memcpy(result.data(), tensor.data_ptr<float>(), sizeof(float) * 16);
  EXPECT_TRUE(expected.isApprox(result, 1e-5f));
}

TEST(CameraUtilsTest, TorchTranslationAndScale) {
  Eigen::Matrix3f R = Eigen::Matrix3f::Identity();
  Eigen::Vector3f t = Eigen::Vector3f::Zero();
  Eigen::Vector3f translate(1.f, 0.f, 0.f);
  float scale = 2.f;
  torch::Tensor tensor = getWorld2View2(R, t, translate, scale);
  Eigen::Matrix4f result;
  std::memcpy(result.data(), tensor.data_ptr<float>(), sizeof(float) * 16);
  Eigen::Matrix4f expected = Eigen::Matrix4f::Identity();
  expected(0, 3) = -2.f;
  EXPECT_TRUE(expected.isApprox(result, 1e-5f));
}

