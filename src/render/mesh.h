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
  std::vector<uint32_t> colors8;
  std::vector<Eigen::Vector4f> extras;
  std::vector<uint32_t> indices;
  MeshData &computeNormals();
  template <class T> MeshData &transform(const T &transform) {
    _transform(Eigen::Affine3f(transform));
    return *this;
  }
  MeshData &append(const MeshData &other);
  MeshData &translate(const Eigen::Vector3f &v);
  MeshData &operator+=(const MeshData &other);
  MeshData &translate(float x, float y, float z);
  MeshData &scale(const Eigen::Vector3f &v);
  MeshData &scale(float x, float y, float z);
  MeshData &scale(float s);
  MeshData &rotate(float angle, const Eigen::Vector3f &axis);
  MeshData &colorize(const Eigen::Vector4f &color);
  MeshData &colorize(float r, float g, float b, float a = 1.0f);
};
MeshData operator+(const MeshData &a, const MeshData &b);
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
  bool _transparent = false;
  std::function<void(MeshData &)> _loader;
  void createBuffer(GLenum type, GLuint index, const void *data, size_t size,
                    size_t stride, size_t element_size = 4,
                    GLenum datatype = GL_FLOAT, bool normalized = false);
  void create();
  void destroy();
  void init();

public:
  Mesh(const MeshData &data);
  Mesh(const std::function<void(MeshData &)> &loader);
  Mesh(const std::function<MeshData()> &loader);
  ~Mesh();
  void bind();
  bool transparent();
  GLuint vertexArrayObject();
  const MeshData &data() const { return _data; }
};
