// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "shapes.h"

#include "../core/log.h"
#include "../render/mesh.h"

#include <geometric_shapes/shape_operations.h>

ShapeDisplay::ShapeDisplay(const std::shared_ptr<Mesh> &mesh) {
  if (mesh) {
    _mesh_renderer = std::make_shared<MeshRenderer>(this, mesh, material());
  }
  //_mesh_renderer->options().primitive_type = GL_LINES;
}

MeshData makePlane() {
  MeshData mesh;

  mesh.indices.emplace_back(0);
  mesh.indices.emplace_back(2);
  mesh.indices.emplace_back(1);
  mesh.indices.emplace_back(0);
  mesh.indices.emplace_back(3);
  mesh.indices.emplace_back(2);

  mesh.positions.emplace_back(-1, -1, 0);
  mesh.positions.emplace_back(-1, 1, 0);
  mesh.positions.emplace_back(1, 1, 0);
  mesh.positions.emplace_back(1, -1, 0);

  mesh.texcoords.emplace_back(0, 0);
  mesh.texcoords.emplace_back(0, 1);
  mesh.texcoords.emplace_back(1, 1);
  mesh.texcoords.emplace_back(1, 0);

  for (size_t i = 0; i < 4; i++) {
    mesh.normals.emplace_back(0, 0, 1);
    mesh.tangents.emplace_back(1, 0, 0);
    mesh.bitangents.emplace_back(0, 1, 0);
  }

  return mesh;
}

MeshData makeBox() {
  MeshData mesh;
  mesh += Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()) *
          Eigen::Translation3f(0, 0, 1) * makePlane();
  mesh += Eigen::Translation3f(0, 0, 1) * makePlane();
  for (int i = 0; i < 4; i++) {
    mesh += Eigen::AngleAxisf(M_PI * 0.5 * i, Eigen::Vector3f::UnitZ()) *
            Eigen::AngleAxisf(M_PI * 0.5, Eigen::Vector3f::UnitX()) *
            Eigen::Translation3f(0, 0, 1) * makePlane();
  }
  return mesh;
}

MeshData makeSphere(size_t segments, size_t rings) {
  MeshData mesh;
  for (size_t y = 0; y <= rings; y++) {
    for (size_t x = 0; x <= segments; x++) {
      float u = x * 1.0 / segments;
      float v = y * 1.0 / rings;
      mesh.texcoords.emplace_back(u, v);
      Eigen::Vector3f n(-std::cos(u * 2 * M_PI) * std::cos((v - 0.5) * M_PI),
                        std::sin(u * 2 * M_PI) * std::cos((v - 0.5) * M_PI),
                        std::sin((v - 0.5) * M_PI));
      mesh.positions.emplace_back(n);
      mesh.normals.emplace_back(n);
    }
  }
  for (size_t y = 0; y < rings; y++) {
    for (size_t x = 0; x < segments; x++) {
      mesh.indices.emplace_back((y + 0) * (segments + 1) + (x + 0));
      mesh.indices.emplace_back((y + 1) * (segments + 1) + (x + 0));
      mesh.indices.emplace_back((y + 0) * (segments + 1) + (x + 1));
      mesh.indices.emplace_back((y + 0) * (segments + 1) + (x + 1));
      mesh.indices.emplace_back((y + 1) * (segments + 1) + (x + 0));
      mesh.indices.emplace_back((y + 1) * (segments + 1) + (x + 1));
    }
  }
  return mesh;
}

MeshData makeDisk(size_t segments) {
  MeshData mesh;
  mesh.positions.emplace_back(0, 0, 0);
  mesh.texcoords.emplace_back(0.5, 0.5);
  mesh.normals.emplace_back(0, 0, 1);
  for (size_t i = 0; i <= segments; i++) {
    float a = i * 1.0 / segments;
    float x = std::sin(a * 2 * M_PI);
    float y = std::cos(a * 2 * M_PI);
    mesh.positions.emplace_back(x, y, 0.0);
    mesh.texcoords.emplace_back(x * 0.5 + 0.5, y * 0.5 + 0.5);
    mesh.normals.emplace_back(0, 0, 1);
  }
  for (size_t i = 0; i < segments; i++) {
    mesh.indices.emplace_back(0);
    mesh.indices.emplace_back(i + 2);
    mesh.indices.emplace_back(i + 1);
  }
  return mesh;
}

MeshData makeCylinder(size_t segments) {
  MeshData mesh;
  for (int s : {-1, 1}) {
    for (size_t i = 0; i <= segments; i++) {
      float a = i * 1.0 / segments;
      float x = std::sin(a * 2 * M_PI);
      float y = std::cos(a * 2 * M_PI);
      mesh.positions.emplace_back(x, y, s);
      mesh.texcoords.emplace_back(a, s * 0.5 + 0.5);
      mesh.normals.emplace_back(x, y, 0);
    }
  }
  for (size_t i = 0; i < segments; i++) {
    mesh.indices.emplace_back(i + 0);
    mesh.indices.emplace_back(segments + 1 + i + 0);
    mesh.indices.emplace_back(segments + 1 + i + 1);
    mesh.indices.emplace_back(i + 0);
    mesh.indices.emplace_back(segments + 1 + i + 1);
    mesh.indices.emplace_back(i + 1);
  }
  mesh += Eigen::Translation3f(0.0f, 0.0f, 1.0f) * makeDisk(segments);
  mesh += Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()) *
          Eigen::Translation3f(0.0f, 0.0f, 1.0f) * makeDisk(segments);
  return mesh;
}

MeshData makeCone(size_t segments) {
  MeshData mesh;
  mesh.positions.emplace_back(0, 0, 1);
  mesh.texcoords.emplace_back(0.5, 0.5);
  mesh.normals.emplace_back(0, 0, 0);
  for (size_t i = 0; i <= segments; i++) {
    float a = i * 1.0 / segments;
    float x = std::sin(a * 2 * M_PI);
    float y = std::cos(a * 2 * M_PI);
    mesh.positions.emplace_back(x, y, 0.0);
    mesh.texcoords.emplace_back(x * 0.5 + 0.5, y * 0.5 + 0.5);
    mesh.normals.emplace_back(x, y, 1);
  }
  for (size_t i = 0; i < segments; i++) {
    mesh.indices.emplace_back(0);
    mesh.indices.emplace_back(i + 2);
    mesh.indices.emplace_back(i + 1);
  }
  mesh +=
      Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()) * makeDisk(segments);
  return mesh;
}

struct PlaneDisplay : ShapeDisplay {
  PlaneDisplay()
      : ShapeDisplay(
            std::make_shared<Mesh>(Eigen::Scaling(0.5f) * makePlane())) {}
  PROPERTY(SizeVector2d, size, SizeVector2d(10, 10));
  virtual void renderSync(const RenderSyncContext &context) override {
    _mesh_renderer->pose(
        Eigen::Affine3d(Eigen::Scaling(size().x(), size().y(), 1.0)));
    ShapeDisplay::renderSync(context);
  }
};
DECLARE_TYPE(PlaneDisplay, ShapeDisplay);

struct DiskDisplay : ShapeDisplay {
  DiskDisplay() {}
  PROPERTY(double, radius, 1.0, min = 0.0);
  PROPERTY(int, segments, 32, min = 3);
  virtual void renderSync(const RenderSyncContext &context) override {
    if (_watcher.changed(segments())) {
      LOG_DEBUG("make sphere");
      _mesh_renderer = std::make_shared<MeshRenderer>(
          this, std::make_shared<Mesh>(makeDisk(segments())), material());
    }
    _mesh_renderer->pose(Eigen::Affine3d(Eigen::Scaling(radius())));
    ShapeDisplay::renderSync(context);
  }
};
DECLARE_TYPE(DiskDisplay, ShapeDisplay);

struct BoxDisplay : ShapeDisplay {
  BoxDisplay()
      : ShapeDisplay(std::make_shared<Mesh>(Eigen::Scaling(0.5f) * makeBox())) {
  }
  PROPERTY(SizeVector3d, size);
  virtual void renderSync(const RenderSyncContext &context) override {
    _mesh_renderer->pose(Eigen::Affine3d(Eigen::Scaling(size())));
    ShapeDisplay::renderSync(context);
  }
};
DECLARE_TYPE(BoxDisplay, ShapeDisplay);

struct SphereDisplay : ShapeDisplay {
  SphereDisplay() {}
  PROPERTY(double, radius, 0.5, min = 0.0);
  PROPERTY(int, rings, 16, min = 3);
  PROPERTY(int, segments, 32, min = 3);
  virtual void renderSync(const RenderSyncContext &context) override {
    if (_watcher.changed(segments(), rings())) {
      _mesh_renderer = std::make_shared<MeshRenderer>(
          this, std::make_shared<Mesh>(makeSphere(segments(), rings())),
          material());
    }
    _mesh_renderer->pose(Eigen::Affine3d(Eigen::Scaling(radius())));
    ShapeDisplay::renderSync(context);
  }
};
DECLARE_TYPE(SphereDisplay, ShapeDisplay);

struct CylinderDisplay : ShapeDisplay {
  CylinderDisplay() {}
  PROPERTY(double, radius, 0.5, min = 0.0);
  PROPERTY(double, height, 1.0, min = 1e-6);
  PROPERTY(int, segments, 32, min = 3);
  virtual void renderSync(const RenderSyncContext &context) override {
    if (_watcher.changed(segments())) {
      _mesh_renderer = std::make_shared<MeshRenderer>(
          this, std::make_shared<Mesh>(makeCylinder(segments())), material());
    }
    _mesh_renderer->pose(
        Eigen::Affine3d(Eigen::Scaling(radius(), radius(), height())));
    ShapeDisplay::renderSync(context);
  }
};
DECLARE_TYPE(CylinderDisplay, ShapeDisplay);

struct ConeDisplay : ShapeDisplay {
  ConeDisplay() {}
  PROPERTY(double, radius, 0.5, min = 0.0);
  PROPERTY(double, height, 1.0, min = 1e-6);
  PROPERTY(int, segments, 32, min = 3);
  virtual void renderSync(const RenderSyncContext &context) override {
    if (_watcher.changed(segments())) {
      _mesh_renderer = std::make_shared<MeshRenderer>(
          this, std::make_shared<Mesh>(makeCone(segments())), material());
    }
    _mesh_renderer->pose(
        Eigen::Affine3d(Eigen::Scaling(radius(), radius(), height())));
    ShapeDisplay::renderSync(context);
  }
};
DECLARE_TYPE(ConeDisplay, ShapeDisplay);
