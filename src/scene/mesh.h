// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include <memory>

#include <Eigen/Dense>

#include "../render/renderlist.h"

#include "node.h"

class MaterialRenderer;
class Mesh;
class RenderAsyncContext;
class RenderSyncContext;
class MeshDisplayBase;
class Material;
class MaterialOverride;
class RenderOptions;
class Interaction;

class MeshRenderer : public SceneNode {
protected:
  std::shared_ptr<MaterialRenderer> _material;
  std::shared_ptr<Mesh> _mesh;
  RenderOptions _render_options, _render_options_2;
  std::function<void(const Interaction &)> _interact_callback;
  Eigen::Affine3d _parent_pose = Eigen::Affine3d::Identity();

public:
  MeshRenderer(const MeshRenderer &) = delete;
  MeshRenderer &operator=(const MeshRenderer &) = delete;
  MeshRenderer(const std::shared_ptr<Mesh> &mesh,
               const std::shared_ptr<MaterialRenderer> &material);
  MeshRenderer(
      const std::shared_ptr<Mesh> &mesh,
      std::shared_ptr<const Material> material,
      std::shared_ptr<const MaterialOverride> material_override = nullptr);
  MeshRenderer(const std::shared_ptr<Mesh> &mesh,
               std::shared_ptr<const Material> material,
               const Eigen::Isometry3d &pose);
  MeshRenderer(
      const std::shared_ptr<Mesh> &mesh,
      std::shared_ptr<const Material> material,
      const std::function<void(const Interaction &)> &interact_callback);
  ~MeshRenderer();
  virtual bool pick(uint32_t id) const override;
  virtual bool interact(const Interaction &interaction) override;
  virtual void renderSync(const RenderSyncContext &context) override;
  virtual void renderAsync(const RenderAsyncContext &context) override;
  const std::shared_ptr<Mesh> &mesh() const;
  const std::shared_ptr<MaterialRenderer> &materialRenderer();
  void options(const RenderOptions &options);
  const RenderOptions &options() const;
  RenderOptions &options();
};

class InstancedMeshRenderer : public MeshRenderer {
  std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>
      _poses;

public:
  InstancedMeshRenderer(const std::shared_ptr<Mesh> &mesh,
                        std::shared_ptr<const Material> material);
  void clearInstances();
  void addInstance(const Eigen::Isometry3d &pose);
  virtual void renderAsync(const RenderAsyncContext &context) override;
};
