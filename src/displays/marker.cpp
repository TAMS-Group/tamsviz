// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "marker.h"

#include "../core/profiler.h"
#include "../core/transformer.h"
#include "../core/workspace.h"
#include "shapes.h"
#include "text.h"

#include <geometric_shapes/shapes.h>

#include <eigen_conversions/eigen_msg.h>

inline Eigen::Vector3d toVector3d(const geometry_msgs::Vector3 &p) {
  return Eigen::Vector3d(p.x, p.y, p.z);
}

inline Eigen::Vector3d toVector3d(const geometry_msgs::Point &p) {
  return Eigen::Vector3d(p.x, p.y, p.z);
}

inline Eigen::Vector3f toVector3f(const geometry_msgs::Point &p) {
  return Eigen::Vector3f(p.x, p.y, p.z);
}

inline Eigen::Vector3f toVector3f(const geometry_msgs::Vector3 &p) {
  return Eigen::Vector3f(p.x, p.y, p.z);
}

inline Eigen::Vector4f toVector4f(const std_msgs::ColorRGBA &p) {
  return Color4(p.r, p.g, p.b, p.a).toLinearVector4f();
}

inline Eigen::Vector4d toVector4d(const std_msgs::ColorRGBA &p) {
  return Color4(p.r, p.g, p.b, p.a).toLinearVector4f().cast<double>();
}

inline Color4 toColor4(const std_msgs::ColorRGBA &p) {
  return Color4(p.r, p.g, p.b, p.a);
}

void VisualizationMarker::update(const visualization_msgs::Marker &marker) {
  std::unique_lock<std::mutex> lock(_mutex);
  PROFILER();
  std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>
      points;
  points.reserve(marker.points.size());
  for (auto &p : marker.points) {
    points.emplace_back(toVector3f(p));
  }
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>
      colors;
  colors.reserve(marker.colors.size());
  for (auto &c : marker.colors) {
    colors.emplace_back(toVector4f(c));
  }
  Eigen::Vector4f color = toVector4f(marker.color);
  _pose.setIdentity();
  _frame = marker.header.frame_id;
  bool type_changed = (_type != marker.type);
  _type = marker.type;
  _color = Color4(1, 1, 1, 1);
  _render_options = RenderOptions();
  _text.clear();
  double radius = marker.scale.x * 0.5;
  switch (marker.type) {
  case visualization_msgs::Marker::POINTS:
  case visualization_msgs::Marker::SPHERE_LIST: {
    if (_mesh_watcher.changed(marker.type, points, colors, color, radius)) {
      _mesh = std::make_shared<Mesh>([marker, radius](MeshData &mesh) {
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
          Eigen::Vector4f extra(radius, radius, 1.0, 0.0);
          // if (marker.type == visualization_msgs::Marker::SPHERE_LIST) {
          // extra.w() = 1.0;
          // }
          mesh.extras.emplace_back(extra);
          mesh.extras.emplace_back(extra);
          mesh.extras.emplace_back(extra);
          mesh.extras.emplace_back(extra);
          mesh.texcoords.emplace_back(-1, -1);
          mesh.texcoords.emplace_back(+1, -1);
          mesh.texcoords.emplace_back(+1, +1);
          mesh.texcoords.emplace_back(-1, +1);
        }
      });
    }
    break;
  }
  case visualization_msgs::Marker::CUBE_LIST: {
    if (_mesh_watcher.changed(marker.type, points, colors, color, radius)) {
      _mesh = std::make_shared<Mesh>([marker, points, colors, color,
                                      radius](MeshData &mesh) {
        static MeshData cube = makeBox();
        Eigen::Vector3f scale = toVector3f(marker.scale) * 0.5f;
        {
          size_t n = cube.positions.size() * points.size();
          mesh.positions.reserve(n);
          mesh.colors.reserve(n);
          mesh.normals.reserve(n);
        }
        {
          size_t n = cube.indices.size() * points.size();
          mesh.indices.reserve(n);
        }
        for (size_t index = 0; index < points.size(); index++) {
          /*MeshData c = cube;
          c.scale(scale);
          c.colorize(index < colors.size() ? colors[index] : color);
          c.translate(points[index]);
          mesh += c;*/
          for (size_t i : cube.indices) {
            mesh.indices.push_back(mesh.positions.size() + i);
          }
          for (auto &p : cube.positions) {
            mesh.positions.emplace_back(p.cwiseProduct(scale) + points[index]);
          }
          for (auto &n : cube.normals) {
            mesh.normals.emplace_back(n);
          }
          if (colors.size() >= points.size()) {
            for (auto &c : cube.positions) {
              mesh.colors.push_back(colors[index]);
            }
          } else {
            for (auto &c : cube.positions) {
              mesh.colors.push_back(color);
            }
          }
        }
      });
    }
    break;
  }
  case visualization_msgs::Marker::LINE_LIST: {
    if (_mesh_watcher.changed(marker.type, points, colors, color, radius)) {
      _mesh = std::make_shared<Mesh>([marker, radius](MeshData &mesh) {
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
      });
    }
    break;
  }
  case visualization_msgs::Marker::LINE_STRIP: {
    if (_mesh_watcher.changed(marker.type, points, colors, color, radius)) {
      _mesh = std::make_shared<Mesh>([marker, radius](MeshData &mesh) {
        for (size_t point_index = 0; point_index + 1 < marker.points.size();
             point_index++) {
          mesh.indices.push_back(mesh.positions.size() + 0);
          mesh.indices.push_back(mesh.positions.size() + 1);
          mesh.indices.push_back(mesh.positions.size() + 2);
          mesh.indices.push_back(mesh.positions.size() + 1);
          mesh.indices.push_back(mesh.positions.size() + 3);
          mesh.indices.push_back(mesh.positions.size() + 2);
          auto a = toVector3f(marker.points.at(point_index + 0));
          auto b = toVector3f(marker.points.at(point_index + 1));
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
              (point_index + 0 < marker.colors.size())
                  ? toVector4f(marker.colors.at(point_index + 0))
                  : toVector4f(marker.color);
          Eigen::Vector4f cb =
              (point_index + 1 < marker.colors.size())
                  ? toVector4f(marker.colors.at(point_index + 1))
                  : toVector4f(marker.color);
          mesh.colors.push_back(ca);
          mesh.colors.push_back(ca);
          mesh.colors.push_back(cb);
          mesh.colors.push_back(cb);
        }
      });
    }
    break;
  }
  case visualization_msgs::Marker::CUBE:
    _mesh_watcher.changed();
    static auto box_mesh = std::make_shared<Mesh>(makeBox());
    _mesh = box_mesh;
    _pose = Eigen::Scaling(toVector3d(marker.scale) * 0.5);
    _color = toColor4(marker.color);
    break;
  case visualization_msgs::Marker::SPHERE:
    _mesh_watcher.changed();
    static auto sphere_mesh = std::make_shared<Mesh>(makeSphere(32, 16));
    _mesh = sphere_mesh;
    _pose = Eigen::Scaling(toVector3d(marker.scale) * 0.5);
    _color = toColor4(marker.color);
    break;
  case visualization_msgs::Marker::CYLINDER:
    _mesh_watcher.changed();
    static auto cylinder_mesh = std::make_shared<Mesh>(makeCylinder(32));
    _mesh = cylinder_mesh;
    _pose = Eigen::Scaling(toVector3d(marker.scale) * 0.5);
    _color = toColor4(marker.color);
    break;
  case visualization_msgs::Marker::ARROW: {
    PROFILER("ARROW");
    if (points.size() >= 2) {
      if (_mesh_watcher.changed(marker.type, points, marker.scale.x,
                                marker.scale.y, marker.scale.z)) {
        Eigen::Vector3d a = points.at(0).cast<double>();
        Eigen::Vector3d b = points.at(1).cast<double>();
        _mesh = std::make_shared<Mesh>([marker, a, b](MeshData &mesh) {
          PROFILER("ARROW");
          double len = (a - b).norm();
          Eigen::Affine3d pose = Eigen::Affine3d(
              Eigen::Translation3d(a) *
              Eigen::AngleAxisd(Eigen::Quaterniond::FromTwoVectors(
                  Eigen::Vector3d::UnitX(), b - a)) *
              Eigen::Scaling(Eigen::Vector3d(len, len * 0.5, len * 0.5)));
          float head_length = 0.23;
          if (marker.scale.z > 0) {
            head_length = std::max(0.0, std::min(marker.scale.z / len, 1.0));
          }
          mesh =
              Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitY()) *
              (Eigen::Translation3f(0.0f, 0.0f, (1.0f - head_length) * 0.5f) *
                   Eigen::Scaling(0.5f, 0.5f, (1.0f - head_length) * 0.5f) *
                   makeCylinder(32) +
               Eigen::Translation3f(0.0f, 0.0f, (1.0f - head_length)) *
                   Eigen::Scaling(1.0f, 1.0f, head_length) * makeCone(32));
          mesh = pose * mesh;
        });
      }
    } else {
      if (_mesh_watcher.changed(marker.type)) {
        static auto arrow_mesh = std::make_shared<Mesh>(
            Eigen::AngleAxisf(M_PI / 2, Eigen::Vector3f::UnitY()) *
            (Eigen::Translation3f(0.0f, 0.0f, 0.77f * 0.5f) *
                 Eigen::Scaling(0.5f, 0.5f, 0.77f * 0.5f) * makeCylinder(32) +
             Eigen::Translation3f(0.0f, 0.0f, 0.77f) *
                 Eigen::Scaling(1.0f, 1.0f, 0.23f) * makeCone(32)));
        _mesh = arrow_mesh;
      }
      _pose = Eigen::Scaling(toVector3d(marker.scale));
    }
    _color = toColor4(marker.color);
    break;
  }
  case visualization_msgs::Marker::TRIANGLE_LIST: {
    PROFILER("TRIANGLE_LIST");
    if (_mesh_watcher.changed(marker.type, points, colors, color)) {
      _mesh = std::make_shared<Mesh>([points, colors, color](MeshData &mesh) {
        PROFILER("TRIANGLE_LIST");
        // throw std::runtime_error("");
        LOG_DEBUG("triangle list " << points.size() << " " << colors.size());
        size_t triangle_count = points.size() / 3;
        size_t vertex_count = triangle_count * 3;
        if (colors.size() >= vertex_count) {
          for (size_t vertex_index = 0; vertex_index < vertex_count;
               vertex_index++) {
            mesh.positions.push_back(points[vertex_index]);
            mesh.colors.push_back(colors[vertex_index]);
          }
        } else if (colors.size() >= triangle_count) {
          for (size_t vertex_index = 0; vertex_index < vertex_count;
               vertex_index++) {
            mesh.positions.push_back(points[vertex_index]);
            mesh.colors.push_back(colors[vertex_index / 3]);
          }
        } else {
          for (size_t vertex_index = 0; vertex_index < vertex_count;
               vertex_index++) {
            mesh.positions.push_back(points[vertex_index]);
            mesh.colors.push_back(color);
          }
        }
        mesh.computeNormals();
      });
    }
    _pose = Eigen::Scaling(toVector3d(marker.scale));
    _render_options.double_sided = true;
    break;
  }
  case visualization_msgs::Marker::TEXT_VIEW_FACING: {
    PROFILER("TEXT_VIEW_FACING");
    _mesh_watcher.changed();
    _text = marker.text;
    _mesh = nullptr;
    break;
  }
  default:
    _mesh_watcher.changed();
    LOG_WARN_THROTTLE(1.0, "unknown marker type " << marker.type);
    _mesh = nullptr;
    break;
  }
  _scale = marker.scale.x;
  Eigen::Affine3d pose;
  tf::poseMsgToEigen(marker.pose, pose);
  _pose = pose * _pose;
}

void VisualizationMarker::renderSync(const RenderSyncContext &context) {
  {
    std::unique_lock<std::mutex> lock(_mutex);
    if (!_material) {
      _material = std::make_shared<Material>();
    }
    _material->color().r() = _color.r();
    _material->color().g() = _color.g();
    _material->color().b() = _color.b();
    _material->opacity() = _color.a();
    if (_mesh) {
      auto mesh_renderer = std::dynamic_pointer_cast<MeshRenderer>(_renderer);
      if (!mesh_renderer || _mesh != mesh_renderer->mesh()) {
        _renderer = create<MeshRenderer>(_mesh, _material, _material_override);
      }
    } else if (_text.size()) {
      auto text_renderer = std::dynamic_pointer_cast<TextRenderer>(_renderer);
      if (!text_renderer) {
        _renderer = create<TextRenderer>(_material);
      }
    } else {
      _renderer = nullptr;
    }
    if (_renderer) {
      _renderer->frame(_frame);
      _renderer->pose(_pose);
      if (auto r = std::dynamic_pointer_cast<MeshRenderer>(_renderer)) {
        r->options() = _render_options;
        // r->materialRenderer()->block().flags |= 4;
      }
      if (auto r = std::dynamic_pointer_cast<TextRenderer>(_renderer)) {
        r->text(_text);
        r->size(_scale);
      }
    }
  }
  SceneNode::renderSync(context);
}

void VisualizationMarkerArray::_update_nolock(
    const visualization_msgs::Marker &marker) {
  if (marker.action == visualization_msgs::Marker::ADD ||
      marker.action == visualization_msgs::Marker::MODIFY) {
    auto &m = _markers[std::make_pair(marker.ns, marker.id)];
    if (!m) {
      m = create<VisualizationMarker>(_material_override);
    }
    m->update(marker);
  }
  if (marker.action == visualization_msgs::Marker::DELETE) {
    _markers.erase(std::make_pair(marker.ns, marker.id));
  }
  if (marker.action == visualization_msgs::Marker::DELETEALL) {
    _markers.clear();
  }
}

void VisualizationMarkerArray::update(
    const visualization_msgs::Marker &marker) {
  std::unique_lock<std::mutex> lock(_mutex);
  _update_nolock(marker);
}

void VisualizationMarkerArray::update(
    const visualization_msgs::MarkerArray &marker_array) {
  std::unique_lock<std::mutex> lock(_mutex);
  for (auto &marker : marker_array.markers) {
    _update_nolock(marker);
  }
}

void VisualizationMarkerArray::renderSync(const RenderSyncContext &context) {
  std::unique_lock<std::mutex> lock(_mutex);
  /*for (auto &p : _markers) {
    LOG_DEBUG("marker " << p.first.first << " " << p.first.second);
  }
  LOG_DEBUG("");*/
  _marker_list.clear();
  for (auto &p : _markers) {
    _marker_list.push_back(p.second);
  }
  SceneNode::renderSync(context);
}

MarkerDisplay::MarkerDisplay() {
  auto *r = _marker_array.get();
  topic().connect(
      _marker_array,
      [r](const std::shared_ptr<const visualization_msgs::Marker> &message) {
        // LOG_DEBUG("marker message");
        r->update(*message);
      });
}

MarkerArrayDisplay::MarkerArrayDisplay() {
  auto *r = _marker_array.get();
  topic().connect(
      _marker_array,
      [r](const std::shared_ptr<const visualization_msgs::MarkerArray>
              &message) { r->update(*message); });
}
