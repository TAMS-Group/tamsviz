// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include <Eigen/Dense>

Eigen::Matrix4f projectionMatrix(double fov_y, double aspect, double near,
                                 double far);

Eigen::MatrixXf lookatMatrix(const Eigen::Vector3d &eye,
                             const Eigen::Vector3d &at,
                             const Eigen::Vector3d &up);
