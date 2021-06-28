// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <Eigen/Dense>

Eigen::Matrix4d projectionMatrix(double fov_y, double aspect, double near,
                                 double far);

Eigen::Matrix4d lookatMatrix(const Eigen::Vector3d &eye,
                             const Eigen::Vector3d &at,
                             const Eigen::Vector3d &up);
