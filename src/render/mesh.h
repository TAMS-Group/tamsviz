// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "opengl.h"
#include "resource.h"

#include <Eigen/Dense>

struct MeshData {
private:
  void _transform(const Eigen::Affine3f &transform);

public:
  std::vector<Eigen::Vector3f> positions;
  std::vector<Eigen::Vector3f> normals;
  std::vector<Eigen::Vector2f> texcoords;
  std::vector<Eigen::Vector3f> tangents;
  std::vector<Eigen::Vector3f> bitangents;
  std::vector<Eigen::Vector4f> colors;
  std::vector<Eigen::Vector4f> extras;
  std::vector<uint32_t> indices;
  MeshData &computeNormals();
  template <class T> MeshData &transform(const T &transform) {
    _transform(Eigen::Affine3f(transform));
    return *this;
  }
  MeshData &append(const MeshData &other);
  inline MeshData &operator+=(const MeshData &other) { return append(other); }
  MeshData &translate(const Eigen::Vector3f &v);
  MeshData &translate(float x, float y, float z) {
    return translate(Eigen::Vector3f(x, y, z));
  }
  MeshData &scale(const Eigen::Vector3f &v) {
    return transform(Eigen::Scaling(v));
  }
  MeshData &scale(float x, float y, float z) {
    return scale(Eigen::Vector3f(x, y, z));
  }
  MeshData &rotate(float angle, const Eigen::Vector3f &axis) {
    return transform(Eigen::AngleAxisf(angle, axis));
  }
  MeshData &colorize(const Eigen::Vector4f &color);
  MeshData &colorize(float r, float g, float b, float a = 1.0f) {
    return colorize(Eigen::Vector4f(r, g, b, a));
  }
};
inline MeshData operator+(const MeshData &a, const MeshData &b) {
  MeshData r = a;
  r.append(b);
  return r;
}
template <class T>
inline MeshData operator*(const T &transform, const MeshData &mesh) {
  MeshData ret = mesh;
  ret.transform(transform);
  return ret;
}

class Mesh : public ResourceBase {
  bool _destructed = false;
  MeshData _data;
  GLuint _vao = 0;
  void createBuffer(GLenum type, GLuint index, const void *data, size_t size,
                    size_t stride);
  void create();
  void destroy();

public:
  Mesh(const MeshData &data);
  ~Mesh();
  void bind();
  GLuint vertexArrayObject();
  const MeshData &data() const { return _data; }
};
