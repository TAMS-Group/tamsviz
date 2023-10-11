// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include "frame.h"
#include "text.h"

#include "../core/topic.h"
#include "../core/watcher.h"
#include "../render/mesh.h"
#include "../scene/material.h"
#include "../scene/mesh.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <unordered_map>

class VisualizationMarker : public SceneNode {
  std::mutex _mutex;
  int _type = -1;
  Eigen::Affine3d _pose = Eigen::Affine3d::Identity();
  std::shared_ptr<Mesh> _mesh;
  Color4 _color;
  std::string _frame;
  RenderOptions _render_options;
  std::shared_ptr<Material> _material;
  std::shared_ptr<SceneNode> _renderer;
  std::string _text;
  Watcher _mesh_watcher;
  std::shared_ptr<MaterialOverride> _material_override;
  double _scale = 1.0;
  uint32_t _material_flags = 0;

 public:
  void update(const visualization_msgs::Marker &marker);
  virtual void renderSync(const RenderSyncContext &context) override;
  VisualizationMarker() {}
  VisualizationMarker(
      const std::shared_ptr<MaterialOverride> &material_override)
      : _material_override(material_override) {}
};

class VisualizationMarkerArray : public SceneNode {
  std::mutex _mutex;
  std::map<std::pair<std::string, int>, std::shared_ptr<VisualizationMarker>>
      _markers;
  std::vector<std::shared_ptr<VisualizationMarker>> _marker_list;
  std::shared_ptr<MaterialOverride> _material_override;
  void _update_nolock(const visualization_msgs::Marker &marker);

 public:
  void update(const visualization_msgs::Marker &marker);
  void update(const visualization_msgs::MarkerArray &marker_array);
  virtual void renderSync(const RenderSyncContext &context) override;
  VisualizationMarkerArray(
      const std::shared_ptr<MaterialOverride> &material_override)
      : _material_override(material_override) {}
};

struct MarkerMaterialOverride : MaterialOverride {
  PROPERTY(bool, vertexGlow, false);
  virtual void applySync(MaterialBlock &block) const override;
};
DECLARE_TYPE(MarkerMaterialOverride, MaterialOverride);

class MarkerDisplayBase : public MeshDisplayBase {
  //  public:
  //   PROPERTY(std::shared_ptr<MarkerOptions>, markerOptions,
  //            std::make_shared<MarkerOptions>());

 protected:
  std::shared_ptr<MarkerMaterialOverride> _material_override =
      std::make_shared<MarkerMaterialOverride>();
  std::shared_ptr<VisualizationMarkerArray> _marker_array =
      node()->create<VisualizationMarkerArray>(_material_override);
  void update(const visualization_msgs::Marker &marker) {
    _marker_array->update(marker);
  }
  void update(const visualization_msgs::MarkerArray &marker_array) {
    _marker_array->update(marker_array);
  }
  MarkerDisplayBase() {}

 public:
  PROPERTY(std::shared_ptr<MarkerMaterialOverride>, materialOverride,
           _material_override);
};
DECLARE_TYPE(MarkerDisplayBase, MeshDisplayBase);

class MarkerFrameDisplayBase : public GenericFrameDisplay<MarkerDisplayBase> {
 public:
  // PROPERTY(Pose, transform);
};

class MarkerDisplay : public MarkerDisplayBase {
 public:
  PROPERTY(TopicProperty<visualization_msgs::Marker>, topic,
           "visualization_marker");
  MarkerDisplay();
};
DECLARE_TYPE_C(MarkerDisplay, MarkerDisplayBase, Marker);

class MarkerArrayDisplay : public MarkerDisplayBase {
 public:
  PROPERTY(TopicProperty<visualization_msgs::MarkerArray>, topic,
           "visualization_marker_array");
  MarkerArrayDisplay();
};
DECLARE_TYPE_C(MarkerArrayDisplay, MarkerDisplayBase, Marker);
