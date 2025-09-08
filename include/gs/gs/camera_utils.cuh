#pragma once

#include <Eigen/Dense>

inline Eigen::Matrix4f getWorld2View2Eigen(
    const Eigen::Matrix3f& R,
    const Eigen::Vector3f& t,
    const Eigen::Vector3f& translate = Eigen::Vector3f::Zero(),
    float scale = 1.0f) {
  Eigen::Matrix4f Rt = Eigen::Matrix4f::Identity();
  Rt.block<3, 3>(0, 0) = R.transpose();
  Rt.block<3, 1>(0, 3) = -R.transpose() * t;

  Eigen::Matrix4f C2W = Rt.inverse();
  Eigen::Vector3f cam_center = C2W.block<3, 1>(0, 3);
  cam_center = (cam_center + translate) * scale;
  C2W.block<3, 1>(0, 3) = cam_center;
  return C2W.inverse();
}

