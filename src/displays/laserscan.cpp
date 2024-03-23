// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "laserscan.h"

#include "../core/log.h"

void LaserScanDisplay::renderSync(const RenderSyncContext &context) {
  auto message = topic().message();
  double radius = pointRadius();
  _material->color().r() = color().r();
  _material->color().g() = color().g();
  _material->color().b() = color().b();
  _material->opacity() = color().a();
  if (message) {
    node()->frame(message->header.frame_id);
  }
  if (_watcher.changed(message, radius)) {
    // LOG_DEBUG("laserscan changed");
    if (message) {
      auto projector = _projector;
      _mesh_renderer = node()->create<MeshRenderer>(
          std::make_shared<Mesh>([message, radius, projector]() {
            sensor_msgs::PointCloud cloud;
            projector->projectLaser(*message, cloud);

            MeshData mesh;
            for (auto &point : cloud.points) {

              mesh.indices.push_back(mesh.positions.size() + 0);
              mesh.indices.push_back(mesh.positions.size() + 1);
              mesh.indices.push_back(mesh.positions.size() + 2);

              mesh.indices.push_back(mesh.positions.size() + 0);
              mesh.indices.push_back(mesh.positions.size() + 2);
              mesh.indices.push_back(mesh.positions.size() + 3);

              for (size_t i = 0; i < 4; i++) {
                mesh.positions.emplace_back(point.x, point.y, point.z);
              }

              Eigen::Vector4f extra(radius, radius, 1.0, 0.0);
              mesh.extras.emplace_back(extra);
              mesh.extras.emplace_back(extra);
              mesh.extras.emplace_back(extra);
              mesh.extras.emplace_back(extra);

              mesh.texcoords.emplace_back(-1, -1);
              mesh.texcoords.emplace_back(+1, -1);
              mesh.texcoords.emplace_back(+1, +1);
              mesh.texcoords.emplace_back(-1, +1);
            }

            return mesh;
          }),
          _material);
    } else {
      _mesh_renderer.reset();
    }
  }
  MeshDisplayBase::renderSync(context);
}
