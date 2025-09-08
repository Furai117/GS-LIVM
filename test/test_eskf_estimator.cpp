#include <gtest/gtest.h>

#include "liw/eskfEstimator.h"

// Define global variables expected by eskf_utils
bool initial_flag = false;
double G_norm = 9.81;

// Helper to generate initialization IMU data
static std::vector<std::pair<double, std::pair<Eigen::Vector3d, Eigen::Vector3d>>>
createInitImu(double dt, int n) {
  std::vector<std::pair<double, std::pair<Eigen::Vector3d, Eigen::Vector3d>>> meas;
  for (int i = 0; i < n; ++i) {
    double t = i * dt;
    Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc(0.0, 0.0, 9.81);
    meas.push_back({t, {gyr, acc}});
  }
  return meas;
}

TEST(ESKFEstimator, GravityInitialization) {
  eskfEstimator eskf;
  eskf.setAccCov(1e-2);
  eskf.setGyrCov(1e-4);
  eskf.setBiasAccCov(1e-6);
  eskf.setBiasGyrCov(1e-6);

  auto meas = createInitImu(0.01, 25);
  eskf.tryInit(meas);

  EXPECT_TRUE(initial_flag);
  Eigen::Vector3d g = eskf.getGravity();
  EXPECT_NEAR(g.x(), 0.0, 1e-3);
  EXPECT_NEAR(g.y(), 0.0, 1e-3);
  EXPECT_NEAR(g.z(), 9.81, 1e-3);
  EXPECT_NEAR(eskf.getBg().norm(), 0.0, 1e-5);
}

TEST(ESKFEstimator, PropagationAndCovariance) {
  eskfEstimator eskf;
  eskf.setAccCov(1e-2);
  eskf.setGyrCov(1e-4);
  eskf.setBiasAccCov(1e-6);
  eskf.setBiasGyrCov(1e-6);

  auto meas = createInitImu(0.01, 25);
  eskf.tryInit(meas);

  double dt = 0.1;
  for (int i = 0; i < 10; ++i) {
    Eigen::Vector3d acc(1.0, 0.0, 9.81);
    Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
    eskf.predict(dt, acc, gyr);
  }

  Eigen::Vector3d p = eskf.getTranslation();
  Eigen::Vector3d v = eskf.getVelocity();
  EXPECT_NEAR(p.x(), 0.5, 1e-2);
  EXPECT_NEAR(p.y(), 0.0, 1e-3);
  EXPECT_NEAR(p.z(), 0.0, 1e-2);
  EXPECT_NEAR(v.x(), 1.0, 1e-3);
  EXPECT_NEAR(v.y(), 0.0, 1e-3);
  EXPECT_NEAR(v.z(), 0.0, 1e-3);

  Eigen::Matrix<double,17,17> cov = eskf.getCovariance();
  EXPECT_GT((cov - Eigen::Matrix<double,17,17>::Identity()).norm(), 1e-6);
  EXPECT_GT(cov(0,0), 1.0);
}

TEST(ESKFEstimator, BiasSetterGetterAndReset) {
  eskfEstimator eskf;
  eskf.setAccCov(1e-2);
  eskf.setGyrCov(1e-4);
  eskf.setBiasAccCov(1e-6);
  eskf.setBiasGyrCov(1e-6);

  auto meas = createInitImu(0.01, 25);
  eskf.tryInit(meas);

  Eigen::Vector3d ba(0.01, -0.02, 0.03);
  Eigen::Vector3d bg(-0.001, 0.002, -0.003);
  eskf.setBa(ba);
  eskf.setBg(bg);
  EXPECT_TRUE(eskf.getBa().isApprox(ba));
  EXPECT_TRUE(eskf.getBg().isApprox(bg));

  // propagate one step with biased acceleration
  Eigen::Vector3d acc(1.0, 0.0, 9.81);
  Eigen::Vector3d gyr = Eigen::Vector3d::Zero();
  eskf.predict(0.1, acc, gyr);

  eskf.calculateLxly();
  eskf.observePose(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity());
  EXPECT_NEAR(eskf.getTranslation().norm(), 0.0, 1e-3);
  EXPECT_NEAR(eskf.getVelocity().norm(), 0.0, 1e-3);
  EXPECT_TRUE(eskf.getRotation().isApprox(Eigen::Quaterniond::Identity(), 1e-3));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

