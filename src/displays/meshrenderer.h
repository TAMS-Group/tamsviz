// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include <memory>

#include <Eigen/Dense>

class MaterialRenderer;
class Mesh;
class RenderAsyncContext;
class RenderSet;
class RenderSyncContext;
class MeshDisplayBase;
class Material;
class MaterialOverride;
class RenderOptions;

typedef Eigen::Transform<double, 3, Eigen::Isometry, Eigen::DontAlign>
    Isometry3du;

typedef Eigen::Transform<double, 3, Eigen::Affine, Eigen::DontAlign> Affine3du;

class MeshRenderer {
  std::shared_ptr<RenderSet> _render_set;
  struct Data {
    bool _visible = true;
    std::shared_ptr<MaterialRenderer> _material;
    std::shared_ptr<Mesh> _mesh;
    std::shared_ptr<RenderOptions> _render_options;
    std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>
        _poses;
    Isometry3du _parent_pose = Eigen::Isometry3d::Identity();
    void renderSync(const RenderSyncContext &context);
    void renderAsync(const RenderAsyncContext &context);
  };
  std::shared_ptr<Data> _data = std::make_shared<Data>();

public:
  MeshRenderer(const MeshRenderer &) = delete;
  MeshRenderer &operator=(const MeshRenderer &) = delete;
  MeshRenderer(MeshDisplayBase *object, const std::shared_ptr<Mesh> &mesh,
               const std::shared_ptr<MaterialRenderer> &material);
  MeshRenderer(
      MeshDisplayBase *object, const std::shared_ptr<Mesh> &mesh,
      std::shared_ptr<const Material> material,
      std::shared_ptr<const MaterialOverride> material_override = nullptr);
  ~MeshRenderer();
  void pose(const Eigen::Isometry3d &pose) {
    _data->_poses = {Eigen::Affine3d(pose)};
  }
  void pose(const Eigen::Affine3d &pose) { _data->_poses = {pose}; }
  template <class T> void poses(const T &begin, const T &end) {
    _data->_poses.clear();
    for (auto it = begin; it != end; ++it) {
      _data->_poses.push_back(*it);
    }
  }
  void clearInstances() { _data->_poses.clear(); }
  void addInstance(const Eigen::Affine3d &pose) {
    _data->_poses.push_back(pose);
  }
  void addInstance(const Eigen::Isometry3d &pose) {
    _data->_poses.push_back(Eigen::Affine3d(pose));
  }
  void options(const RenderOptions &options);
  const RenderOptions &options() const { return *_data->_render_options; }
  RenderOptions &options() { return *_data->_render_options; }
  inline void show() { _data->_visible = true; }
  inline void hide() { _data->_visible = false; }
  inline void setVisibility(bool v) { _data->_visible = v; }
  friend class MeshDisplayBase;
  friend class RenderSet;
};
