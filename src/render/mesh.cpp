// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "mesh.h"

#include "../core/log.h"
#include "../core/profiler.h"
#include "shader.h"

#include <map>

MeshData operator+(const MeshData &a, const MeshData &b) {
  MeshData r = a;
  r.append(b);
  return r;
}

MeshData &MeshData::colorize(float r, float g, float b, float a) {
  return colorize(Eigen::Vector4f(r, g, b, a));
}

MeshData &MeshData::operator+=(const MeshData &other) { return append(other); }

MeshData &MeshData::translate(float x, float y, float z) {
  return translate(Eigen::Vector3f(x, y, z));
}

MeshData &MeshData::scale(const Eigen::Vector3f &v) {
  return transform(Eigen::Scaling(v));
}

MeshData &MeshData::scale(float x, float y, float z) {
  return scale(Eigen::Vector3f(x, y, z));
}

MeshData &MeshData::scale(float s) { return scale(s, s, s); }

MeshData &MeshData::rotate(float angle, const Eigen::Vector3f &axis) {
  return transform(Eigen::AngleAxisf(angle, axis));
}

void MeshData::_transform(const Eigen::Affine3f &transform) {
  for (auto &p : positions) {
    p = transform * p;
  }
  Eigen::Matrix3f normal_matrix = transform.linear().inverse().transpose();
  for (auto *component : {&normals, &tangents, &bitangents}) {
    for (auto &n : *component) {
      n = (normal_matrix * n).eval();
    }
  }
}

template <class T>
static void appendMeshComponent(const MeshData &mesh, std::vector<T> &a,
                                const std::vector<T> &b, const T &def) {
  if (!b.empty()) {
    if (a.size() < mesh.positions.size()) {
      a.resize(mesh.positions.size(), def);
    }
    a.insert(a.end(), b.begin(), b.end());
  }
}

MeshData &MeshData::append(const MeshData &other) {
  for (auto i : other.indices) {
    indices.push_back(i + positions.size());
  }
  appendMeshComponent(*this, normals, other.normals, Eigen::Vector3f(0, 0, 0));
  appendMeshComponent(*this, texcoords, other.texcoords, Eigen::Vector2f(0, 0));
  appendMeshComponent(*this, tangents, other.tangents,
                      Eigen::Vector3f(0, 0, 0));
  appendMeshComponent(*this, bitangents, other.bitangents,
                      Eigen::Vector3f(0, 0, 0));
  appendMeshComponent(*this, colors, other.colors, Eigen::Vector4f(0, 0, 0, 1));
  appendMeshComponent(*this, colors8, other.colors8, uint32_t(0xff000000));
  appendMeshComponent(*this, extras, other.extras, Eigen::Vector4f(0, 0, 0, 1));
  appendMeshComponent(*this, positions, other.positions,
                      Eigen::Vector3f(0, 0, 0));
  return *this;
}

MeshData &MeshData::translate(const Eigen::Vector3f &v) {
  for (auto &p : positions) {
    p += v;
  }
  return *this;
}

MeshData &MeshData::colorize(const Eigen::Vector4f &color) {
  colors.resize(positions.size());
  for (auto &c : colors) {
    c = color;
  }
  return *this;
}

MeshData &MeshData::computeNormals() {
  if (positions.empty()) {
    return *this;
  }
  if (indices.empty() && !positions.empty()) {
    while (indices.size() < positions.size()) {
      indices.push_back(indices.size());
    }
  }
  normals.clear();
  normals.resize(positions.size(), Eigen::Vector3f::Zero());
  for (size_t face_index = 0; face_index < indices.size() / 3; face_index++) {
    auto &A = positions.at(indices.at(face_index * 3 + 0));
    auto &B = positions.at(indices.at(face_index * 3 + 1));
    auto &C = positions.at(indices.at(face_index * 3 + 2));
    Eigen::Vector3f face_normal = (B - A).cross(C - A);
    for (size_t edge_index = 0; edge_index < 3; edge_index++) {
      size_t index = face_index * 3 + edge_index;
      normals.at(indices.at(index)) += face_normal;
    }
  }
  for (auto &n : normals) {
    n.normalize();
  }
  return *this;
}

Mesh::Mesh(const MeshData &data) : _data(data) { init(); }

Mesh::Mesh(const std::function<void(MeshData &)> &loader) : _loader(loader) {}

Mesh::Mesh(const std::function<MeshData()> &loader)
    : _loader([loader](MeshData &d) { d = loader(); }) {}

void Mesh::init() {
  {
    PROFILER("check mesh transparency");
    _transparent = false;
    if (!_data.colors.empty()) {
      auto *alpha_begin = &(_data.colors[0].w());
      auto *alpha_end = alpha_begin + _data.colors.size() * 4;
      for (auto *alpha_pointer = alpha_begin; alpha_pointer < alpha_end;
           alpha_pointer += 4) {
        if ((*alpha_pointer) < 1.0) {
          _transparent = true;
          break;
        }
      }
    }
  }
}

void Mesh::createBuffer(GLenum type, GLuint index, const void *data,
                        size_t size, size_t stride, size_t element_size,
                        GLenum datatype, bool normalized) {
  GLuint vbo = 0;
  V_GL(glBindVertexArray(_vao));
  V_GL(glGenBuffers(1, &vbo));
  V_GL(glBindBuffer(type, vbo));
  V_GL(glBufferData(type, size, data, GL_STATIC_DRAW));
  if (type == GL_ARRAY_BUFFER) {
    V_GL(glEnableVertexAttribArray(index));
    V_GL(glVertexAttribPointer(index, stride / element_size, datatype,
                               normalized, stride, 0));
  }
  V_GL(glBindVertexArray(0));
  V_GL(glDeleteBuffers(1, &vbo));
}

void Mesh::create() {
  if (!_vao) {
    if (_loader) {
      PROFILER("mesh callback");
      _data = MeshData();
      _loader(_data);
      _loader = nullptr;
      init();
    }
    PROFILER("create mesh buffers");
    V_GL(glGenVertexArrays(1, &_vao));
    if (!_data.positions.empty()) {
      createBuffer(GL_ARRAY_BUFFER, (GLuint)VertexAttributes::position,
                   _data.positions.data(),
                   _data.positions.size() * sizeof(_data.positions[0]),
                   sizeof(_data.positions[0]));
      if (!_data.normals.empty()) {
        createBuffer(GL_ARRAY_BUFFER, (GLuint)VertexAttributes::normal,
                     _data.normals.data(),
                     _data.normals.size() * sizeof(_data.normals[0]),
                     sizeof(_data.normals[0]));
      }
      if (!_data.texcoords.empty()) {
        createBuffer(GL_ARRAY_BUFFER, (GLuint)VertexAttributes::texcoord,
                     _data.texcoords.data(),
                     _data.texcoords.size() * sizeof(_data.texcoords[0]),
                     sizeof(_data.texcoords[0]));
      }
      if (!_data.tangents.empty()) {
        createBuffer(GL_ARRAY_BUFFER, (GLuint)VertexAttributes::tangent,
                     _data.tangents.data(),
                     _data.tangents.size() * sizeof(_data.tangents[0]),
                     sizeof(_data.tangents[0]));
      }
      if (!_data.bitangents.empty()) {
        createBuffer(GL_ARRAY_BUFFER, (GLuint)VertexAttributes::bitangent,
                     _data.bitangents.data(),
                     _data.bitangents.size() * sizeof(_data.bitangents[0]),
                     sizeof(_data.bitangents[0]));
      }
      if (!_data.extras.empty()) {
        createBuffer(GL_ARRAY_BUFFER, (GLuint)VertexAttributes::extra,
                     _data.extras.data(),
                     _data.extras.size() * sizeof(_data.extras[0]),
                     sizeof(_data.extras[0]));
      }
      if (!_data.colors.empty()) {
        createBuffer(GL_ARRAY_BUFFER, (GLuint)VertexAttributes::color,
                     _data.colors.data(),
                     _data.colors.size() * sizeof(_data.colors[0]),
                     sizeof(_data.colors[0]));
      } else if (!_data.colors8.empty()) {
        createBuffer(GL_ARRAY_BUFFER, (GLuint)VertexAttributes::color,
                     _data.colors8.data(),
                     _data.colors8.size() * sizeof(_data.colors8[0]),
                     sizeof(_data.colors8[0]), 1, GL_UNSIGNED_BYTE, true);
      } else {
        V_GL(glVertexAttrib4f((GLuint)VertexAttributes::color, 1, 1, 1, 1));
      }
      if (!_data.indices.empty()) {
        createBuffer(GL_ELEMENT_ARRAY_BUFFER, 0, _data.indices.data(),
                     _data.indices.size() * sizeof(_data.indices[0]),
                     sizeof(_data.indices[0]));
      }
    }
  }
}

void Mesh::destroy() {
  if (_vao) {
    GLuint vao = _vao;
    cleanup([vao]() {
      PROFILER("mesh cleanup");
      V_GL(glDeleteVertexArrays(1, &vao));
    });
    _vao = 0;
  }
}

Mesh::~Mesh() {
  if (_destructed) {
    throw std::runtime_error("mesh already destructed");
  }
  _destructed = true;
  destroy();
}

bool Mesh::transparent() {
  if (!_vao) {
    create();
  }
  return _transparent;
}

GLuint Mesh::vertexArrayObject() {
  if (!_vao) {
    create();
  }
  return _vao;
}

void Mesh::bind() {
  if (!_vao) {
    create();
  }
  V_GL(glBindVertexArray(_vao));
}
