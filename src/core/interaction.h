// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include <Eigen/Dense>

struct InteractionRay {
  int x = 0;
  int y = 0;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d direction = Eigen::Vector3d::Zero();
  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  Eigen::Vector3d projectToAxis(const Eigen::Vector3d &start,
                                const Eigen::Vector3d &direction) const;
  Eigen::Vector3d projectPlane(const Eigen::Vector3d &point,
                               const Eigen::Vector3d &normal) const;
  Eigen::Matrix4d view_matrix = Eigen::Matrix4d::Identity();
  Eigen::Matrix4d projection_matrix = Eigen::Matrix4d::Identity();
};

struct Interaction {
  InteractionRay begin;
  InteractionRay current;
  InteractionRay previous;
  bool finished = false;
  bool pressed = false;
  uint32_t id = 0;
  Eigen::Vector3d projectToAxis(const Eigen::Vector3d &direction) const;
  Eigen::Vector3d projectPlane(const Eigen::Vector3d &normal) const;
};
