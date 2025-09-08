#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <iostream>

// Basic constants used by the estimator
constexpr double THETA_THRESHOLD = 0.0001;
constexpr int MIN_INI_COUNT = 20;
constexpr double MIN_INI_TIME = 0.2;
constexpr double MAX_GYR_VAR = 0.5;
constexpr double MAX_ACC_VAR = 0.6;

// Global flags/values expected by the estimator implementation
extern bool initial_flag;
extern double G_norm;

// Simple stand-ins for ROS and glog macros used in the original project
#ifndef ROS_INFO
#define ROS_INFO(... ) do { (void)0; } while(0)
#endif
#ifndef LOG
#define LOG(level) std::cerr
#endif

struct numType {
  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3> normalizeR(
      const Eigen::MatrixBase<Derived>& R_in) {
    typedef typename Derived::Scalar Scalar_t;
    Eigen::Quaternion<Scalar_t> q(R_in);
    q.normalize();
    return q.toRotationMatrix();
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(
      const Eigen::MatrixBase<Derived>& mat) {
    typedef typename Derived::Scalar Scalar_t;
    Eigen::Matrix<Scalar_t, 3, 3> mat_skew;
    mat_skew << Scalar_t(0), -mat(2), mat(1),
        mat(2), Scalar_t(0), -mat(0),
        -mat(1), mat(0), Scalar_t(0);
    return mat_skew;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 2> derivativeS2(
      const Eigen::MatrixBase<Derived>& g_in) {
    typedef typename Derived::Scalar Scalar_t;
    Eigen::Matrix<Scalar_t, 3, 2> B_x;
    Eigen::Matrix<Scalar_t, 3, 1> g = g_in;
    g.normalize();
    B_x(0, 0) = Scalar_t(1.0) - g(0) * g(0) / (Scalar_t(1.0) + g(2));
    B_x(0, 1) = -g(0) * g(1) / (Scalar_t(1.0) + g(2));
    B_x(1, 0) = B_x(0, 1);
    B_x(1, 1) = Scalar_t(1.0) - g(1) * g(1) / (Scalar_t(1.0) + g(2));
    B_x(2, 0) = -g(0);
    B_x(2, 1) = -g(1);
    return B_x;
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 1> rotationToSo3(
      const Eigen::MatrixBase<Derived>& R_in) {
    typedef typename Derived::Scalar Scalar_t;
    Eigen::Matrix<Scalar_t, 3, 3> R = normalizeR(R_in);
    Scalar_t theta = acos((R(0,0)+R(1,1)+R(2,2)-Scalar_t(1.0))/Scalar_t(2));
    if (theta < Scalar_t(THETA_THRESHOLD)) {
      return Eigen::Matrix<Scalar_t,3,1>(R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1)) / Scalar_t(2.0);
    } else {
      return theta * Eigen::Matrix<Scalar_t,3,1>(R(2,1)-R(1,2), R(0,2)-R(2,0), R(1,0)-R(0,1)) /
             (Scalar_t(2.0)*sin(theta));
    }
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar, 3, 3> so3ToRotation(
      const Eigen::MatrixBase<Derived>& so3_in) {
    typedef typename Derived::Scalar Scalar_t;
    Scalar_t theta = so3_in.norm();
    if (theta < Scalar_t(THETA_THRESHOLD)) {
      Eigen::Matrix<Scalar_t,3,3> u_x = skewSymmetric(so3_in);
      return Eigen::Matrix<Scalar_t,3,3>::Identity() + u_x + Scalar_t(0.5) * u_x * u_x;
    } else {
      Eigen::Matrix<Scalar_t,3,3> u_x = skewSymmetric(so3_in.normalized());
      return Eigen::Matrix<Scalar_t,3,3>::Identity() + sin(theta) * u_x +
             (Scalar_t(1) - cos(theta)) * u_x * u_x;
    }
  }

  template <typename Derived>
  static Eigen::Quaternion<typename Derived::Scalar> so3ToQuat(
      const Eigen::MatrixBase<Derived>& so3_in) {
    typedef typename Derived::Scalar Scalar_t;
    Scalar_t theta = so3_in.norm();
    if (theta < Scalar_t(THETA_THRESHOLD)) {
      Eigen::Matrix<Scalar_t,3,1> half_so3 = so3_in / Scalar_t(2.0);
      Eigen::Quaternion<Scalar_t> q(Scalar_t(1.0), half_so3.x(), half_so3.y(), half_so3.z());
      q.normalize();
      return q;
    } else {
      Eigen::Matrix<Scalar_t,3,1> u = so3_in.normalized();
      Eigen::Quaternion<Scalar_t> q(
          cos(Scalar_t(0.5)*theta),
          u.x()*sin(Scalar_t(0.5)*theta),
          u.y()*sin(Scalar_t(0.5)*theta),
          u.z()*sin(Scalar_t(0.5)*theta));
      q.normalize();
      return q;
    }
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar,3,1> quatToSo3(
      const Eigen::QuaternionBase<Derived>& q_in) {
    return rotationToSo3(q_in.toRotationMatrix());
  }

  template <typename Derived>
  static Eigen::Matrix<typename Derived::Scalar,3,3> invJrightSo3(
      const Eigen::MatrixBase<Derived>& so3_in) {
    typedef typename Derived::Scalar Scalar_t;
    Scalar_t theta = so3_in.norm();
    if (theta < Scalar_t(THETA_THRESHOLD)) {
      return cos(theta/Scalar_t(2)) * Eigen::Matrix<Scalar_t,3,3>::Identity() +
             Scalar_t(0.125) * so3_in * so3_in.transpose() +
             Scalar_t(0.5) * skewSymmetric(so3_in);
    } else {
      Eigen::Matrix<Scalar_t,3,1> u = so3_in.normalized();
      return Scalar_t(0.5)*theta/tan(theta/Scalar_t(2)) * Eigen::Matrix<Scalar_t,3,3>::Identity() +
             (Scalar_t(1) - Scalar_t(0.5)*theta/tan(theta/Scalar_t(2))) * u * u.transpose() +
             Scalar_t(0.5) * skewSymmetric(so3_in);
    }
  }
};

