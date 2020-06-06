// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "marker.h"

#include "shapes.h"

#include "../core/transformer.h"
#include "../core/workspace.h"

#include <geometric_shapes/shapes.h>

#include <eigen_conversions/eigen_msg.h>

inline Eigen::Vector3d toVector3d(const geometry_msgs::Vector3 &p) {
  return Eigen::Vector3d(p.x, p.y, p.z);
}

inline Eigen::Vector3f toVector3f(const geometry_msgs::Point &p) {
  return Eigen::Vector3f(p.x, p.y, p.z);
}

inline Eigen::Vector4f toVector4f(const std_msgs::ColorRGBA &p) {
  return Eigen::Vector4f(p.r, p.g, p.b, p.a);
}

inline Eigen::Vector4d toVector4d(const std_msgs::ColorRGBA &p) {
  return Eigen::Vector4d(p.r, p.g, p.b, p.a);
}

inline Color4 toColor4(const std_msgs::ColorRGBA &p) {
  return Color4(p.r, p.g, p.b, p.a);
}

void VisualizationMarker::update(const visualization_msgs::Marker &marker) {
  _pose.setIdentity();
  _frame = marker.header.frame_id;
  bool type_changed = (_type != marker.type);
  _type = marker.type;
  _color = Color4(1, 1, 1, 1);
  double radius = marker.scale.x * 0.5;
  switch (marker.type) {
  case visualization_msgs::Marker::POINTS:
  case visualization_msgs::Marker::SPHERE_LIST: {
    MeshData mesh;
    for (size_t index = 0; index < marker.points.size(); index++) {
      auto p = toVector3f(marker.points.at(index));
      mesh.indices.push_back(mesh.positions.size() + 0);
      mesh.indices.push_back(mesh.positions.size() + 1);
      mesh.indices.push_back(mesh.positions.size() + 2);
      mesh.indices.push_back(mesh.positions.size() + 0);
      mesh.indices.push_back(mesh.positions.size() + 2);
      mesh.indices.push_back(mesh.positions.size() + 3);
      Eigen::Vector4f color = (index < marker.colors.size())
                                  ? toVector4f(marker.colors.at(index))
                                  : toVector4f(marker.color);
      for (size_t i = 0; i < 4; i++) {
        mesh.positions.push_back(p);
        mesh.colors.push_back(color);
      }
      Eigen::Vector4f extra(radius, 1.0, 1.0, 0.0);
      if (marker.type == visualization_msgs::Marker::SPHERE_LIST) {
        // extra.w() = 1.0;
      }
      mesh.extras.emplace_back(extra);
      mesh.extras.emplace_back(extra);
      mesh.extras.emplace_back(extra);
      mesh.extras.emplace_back(extra);
      mesh.texcoords.emplace_back(-1, -1);
      mesh.texcoords.emplace_back(+1, -1);
      mesh.texcoords.emplace_back(+1, +1);
      mesh.texcoords.emplace_back(-1, +1);
    }
    _mesh = std::make_shared<Mesh>(mesh);
    break;
  }
  case visualization_msgs::Marker::LINE_LIST: {
    MeshData mesh;
    for (size_t line_index = 0; line_index < marker.points.size() / 2;
         line_index++) {
      mesh.indices.push_back(mesh.positions.size() + 0);
      mesh.indices.push_back(mesh.positions.size() + 1);
      mesh.indices.push_back(mesh.positions.size() + 2);
      mesh.indices.push_back(mesh.positions.size() + 1);
      mesh.indices.push_back(mesh.positions.size() + 3);
      mesh.indices.push_back(mesh.positions.size() + 2);
      auto a = toVector3f(marker.points.at(line_index * 2 + 0));
      auto b = toVector3f(marker.points.at(line_index * 2 + 1));
      mesh.positions.push_back(a);
      mesh.positions.push_back(a);
      mesh.positions.push_back(b);
      mesh.positions.push_back(b);
      mesh.tangents.push_back(a - b);
      mesh.tangents.push_back(a - b);
      mesh.tangents.push_back(a - b);
      mesh.tangents.push_back(a - b);
      mesh.extras.emplace_back(+radius, 0.0, 2.0, 0.0);
      mesh.extras.emplace_back(-radius, 0.0, 2.0, 0.0);
      mesh.extras.emplace_back(+radius, 0.0, 2.0, 0.0);
      mesh.extras.emplace_back(-radius, 0.0, 2.0, 0.0);
      Eigen::Vector4f ca =
          (line_index * 2 + 0 < marker.colors.size())
              ? toVector4f(marker.colors.at(line_index * 2 + 0))
              : toVector4f(marker.color);
      Eigen::Vector4f cb =
          (line_index * 2 + 1 < marker.colors.size())
              ? toVector4f(marker.colors.at(line_index * 2 + 1))
              : toVector4f(marker.color);
      mesh.colors.push_back(ca);
      mesh.colors.push_back(ca);
      mesh.colors.push_back(cb);
      mesh.colors.push_back(cb);
    }
    _mesh = std::make_shared<Mesh>(mesh);
    break;
  }
  case visualization_msgs::Marker::CUBE:
    static auto box_mesh = std::make_shared<Mesh>(makeBox());
    _mesh = box_mesh;
    _pose = Eigen::Scaling(toVector3d(marker.scale) * 0.5);
    _color = toColor4(marker.color);
    break;
  case visualization_msgs::Marker::SPHERE:
    static auto sphere_mesh = std::make_shared<Mesh>(makeSphere(32, 16));
    _mesh = sphere_mesh;
    _pose = Eigen::Scaling(toVector3d(marker.scale) * 0.5);
    _color = toColor4(marker.color);
    break;
  case visualization_msgs::Marker::CYLINDER:
    static auto cylinder_mesh = std::make_shared<Mesh>(makeCylinder(32));
    _mesh = cylinder_mesh;
    _pose = Eigen::Scaling(toVector3d(marker.scale) * 0.5);
    _color = toColor4(marker.color);
    break;
  case visualization_msgs::Marker::ARROW:
    static auto arrow_mesh = std::make_shared<Mesh>(
        Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitY()) *
        (Eigen::Translation3f(0.0f, 0.0f, 0.77f / 2) *
             Eigen::Scaling(0.5f, 0.5f, 0.77f * 0.5f) * makeCylinder(32) +
         Eigen::Translation3f(0.0f, 0.0f, 0.77f) *
             Eigen::Scaling(1.0f, 1.0f, 0.23f) * makeCone(32)));
    _mesh = arrow_mesh;
    _pose = Eigen::Scaling(toVector3d(marker.scale));
    _color = toColor4(marker.color);
    break;
  default:
    LOG_WARN_THROTTLE(1.0, "unknown marker type " << marker.type);
    _mesh = nullptr;
    break;
  }
  Eigen::Affine3d pose;
  tf::poseMsgToEigen(marker.pose, pose);
  _pose = pose * _pose;
}

void MarkerDisplayBase::Data::update(const visualization_msgs::Marker &marker) {
  std::lock_guard<std::mutex> lock(_mutex);
  auto &m = _markers[std::make_pair(marker.ns, marker.id)];
  if (!m) {
    m = std::make_shared<VisualizationMarker>(marker);
  } else {
    m->update(marker);
  }
}
void MarkerDisplayBase::update(const visualization_msgs::Marker &marker) {
  _data->update(marker);
}
MarkerDisplayBase::MarkerDisplayBase() {}
void MarkerDisplayBase::renderSync(const RenderSyncContext &c) {
  auto context = c;
  auto parent_pose = context.pose;
  context.pose.setIdentity();
  {
    std::lock_guard<std::mutex> lock(_data->_mutex);
    for (auto &p : _data->_markers) {
      auto &renderer = _renderers[p.first];
      if (!renderer) {
        // LOG_DEBUG("new marker renderer");
        renderer = std::make_shared<Renderer>();
        renderer->material = std::make_shared<Material>();
      }
      renderer->material->color().r() = p.second->color().r();
      renderer->material->color().g() = p.second->color().g();
      renderer->material->color().b() = p.second->color().b();
      renderer->material->opacity() = p.second->color().a();
      if (renderer->mesh != p.second->mesh()) {
        renderer->mesh = p.second->mesh();
        if (renderer->mesh) {
          // LOG_DEBUG("new marker mesh renderer");
          renderer->mesh_renderer = std::make_shared<MeshRenderer>(
              this, renderer->mesh, renderer->material);
        } else {
          renderer->mesh_renderer = nullptr;
        }
      }
      if (renderer->mesh_renderer) {
        if (auto pose = LockScope()->document()->display()->transformer->lookup(
                p.second->frame())) {
          renderer->mesh_renderer->pose(*pose * p.second->pose());
        } else {
          renderer->mesh_renderer->pose(parent_pose * p.second->pose());
        }
      }
    }
    for (auto it = _renderers.begin(); it != _renderers.end();) {
      if (_data->_markers.find(it->first) == _data->_markers.end()) {
        it = _renderers.erase(it);
      } else {
        ++it;
      }
    }
  }
  MeshDisplayBase::renderSync(context);
}
void MarkerDisplayBase::renderAsync(const RenderAsyncContext &context) {
  MeshDisplayBase::renderAsync(context);
}

MarkerDisplay::MarkerDisplay() {
  auto *r = _data.get();
  topic().connect(_data,
                  [r](const std::shared_ptr<visualization_msgs::Marker> &m) {
                    // LOG_DEBUG("marker message");
                    r->update(*m);
                  });
}

MarkerArrayDisplay::MarkerArrayDisplay() {
  auto *r = _data.get();
  topic().connect(
      _data,
      [r](const std::shared_ptr<visualization_msgs::MarkerArray> &message) {
        // LOG_DEBUG("marker array message");
        for (auto &marker : message->markers) {
          r->update(marker);
        }
      });
}
