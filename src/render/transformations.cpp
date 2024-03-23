// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "transformations.h"

Eigen::Matrix4d projectionMatrix(double fov_y, double aspect, double near,
                                 double far) {
  double s = 1.0f / std::tan((double)fov_y * 0.5);
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Zero();
  matrix(0, 0) = s * std::sqrt(aspect);
  matrix(1, 1) = s / std::sqrt(aspect);
  matrix(2, 2) = -far / (far - near);
  matrix(2, 3) = -far * near / (far - near);
  matrix(3, 2) = -1;
  return matrix;
}

Eigen::Matrix4d lookatMatrix(const Eigen::Vector3d &eye,
                             const Eigen::Vector3d &at,
                             const Eigen::Vector3d &up) {
  Eigen::Vector3d z = (eye - at).normalized();
  Eigen::Vector3d x = up.cross(z).normalized();
  Eigen::Vector3d y = z.cross(x).normalized();
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  matrix(0, 0) = x[0];
  matrix(0, 1) = x[1];
  matrix(0, 2) = x[2];
  matrix(1, 0) = y[0];
  matrix(1, 1) = y[1];
  matrix(1, 2) = y[2];
  matrix(2, 0) = z[0];
  matrix(2, 1) = z[1];
  matrix(2, 2) = z[2];
  matrix(0, 3) = -x.dot(eye);
  matrix(1, 3) = -y.dot(eye);
  matrix(2, 3) = -z.dot(eye);
  return matrix;
}

Eigen::Matrix4d cubeMapProjectionMatrix(double near, double far) {
  return projectionMatrix(M_PI / 2, 1, near, far);
}

Eigen::Matrix4d cubeMapViewMatrix(const Eigen::Vector3d &eye, size_t face) {

  static std::array<Eigen::Vector3d, 6> at = {
      Eigen::Vector3d(+1.0f, 0.0f, 0.0f), Eigen::Vector3d(-1.0f, 0.0f, 0.0f),
      Eigen::Vector3d(0.0f, +1.0f, 0.0f), Eigen::Vector3d(0.0f, -1.0f, 0.0f),
      Eigen::Vector3d(0.0f, 0.0f, +1.0f), Eigen::Vector3d(0.0f, 0.0f, -1.0f)};

  static std::array<Eigen::Vector3d, 6> up = {
      Eigen::Vector3d(0.0f, 1.0f, 0.0f),  Eigen::Vector3d(0.0f, 1.0f, 0.0f),
      Eigen::Vector3d(0.0f, 0.0f, -1.0f), Eigen::Vector3d(0.0f, 0.0f, 1.0f),
      Eigen::Vector3d(0.0f, 1.0f, 0.0f),  Eigen::Vector3d(0.0f, 1.0f, 0.0f)};

  return lookatMatrix(eye, at[face] + eye, -up[face]);
}
