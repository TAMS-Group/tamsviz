// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "interaction.h"

Eigen::Vector3d
InteractionRay::projectToAxis(const Eigen::Vector3d &axis_start,
                              const Eigen::Vector3d &axis_direction) const {
  Eigen::Vector3d camera_direction =
      view_matrix.inverse().col(2).head(3).normalized();
  Eigen::Vector3d side_vector =
      camera_direction.cross(axis_direction).normalized();
  Eigen::Vector3d plane_normal = side_vector.cross(axis_direction).normalized();
  Eigen::Hyperplane<double, 3> plane(plane_normal, axis_start);
  Eigen::ParametrizedLine<double, 3> ray(center, direction);
  Eigen::Vector3d intersection = ray.intersectionPoint(plane);
  Eigen::Vector3d d = intersection - axis_start;
  return d - side_vector * d.dot(side_vector) + axis_start;
}

Eigen::Vector3d
Interaction::projectToAxis(const Eigen::Vector3d &direction) const {
  return current.projectToAxis(begin.point, direction) -
         previous.projectToAxis(begin.point, direction);
}

Eigen::Vector3d InteractionRay::projectPlane(const Eigen::Vector3d &p,
                                             const Eigen::Vector3d &n) const {
  Eigen::Hyperplane<double, 3> plane(n, p);
  Eigen::ParametrizedLine<double, 3> ray(center, direction);
  return ray.intersectionPoint(plane);
}

Eigen::Vector3d Interaction::projectPlane(const Eigen::Vector3d &normal) const {
  return current.projectPlane(begin.point, normal) -
         previous.projectPlane(begin.point, normal);
}
