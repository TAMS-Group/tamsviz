// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "frame.h"
#include "material.h"
#include "mesh.h"

#include "../core/topic.h"
#include "../render/mesh.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <unordered_map>

class VisualizationMarker {
  int _type = -1;
  Eigen::Affine3d _pose = Eigen::Affine3d::Identity();
  std::shared_ptr<Mesh> _mesh;
  Color4 _color;
  std::string _frame;

public:
  std::shared_ptr<Mesh> mesh() const { return _mesh; }
  int type() const { return _type; }
  const Eigen::Affine3d &pose() const { return _pose; }
  const Color4 &color() const { return _color; }
  void update(const visualization_msgs::Marker &marker);
  const std::string &frame() const { return _frame; }
  VisualizationMarker() {}
  VisualizationMarker(const visualization_msgs::Marker &marker) {
    update(marker);
  }
};

namespace std {
template <> struct hash<std::pair<std::string, int>> {
  size_t operator()(const std::pair<std::string, int> &p) const {
    return std::hash<std::string>()(p.first) ^ std::hash<int>()(p.second);
  }
};
} // namespace std

class MarkerDisplayBase : public MeshDisplayBase {
protected:
  struct Data {
    std::mutex _mutex;
    std::unordered_map<std::pair<std::string, int>,
                       std::shared_ptr<VisualizationMarker>>
        _markers;
    void update(const visualization_msgs::Marker &marker);
  };
  std::shared_ptr<Data> _data = std::make_shared<Data>();
  struct Renderer {
    std::shared_ptr<Mesh> mesh;
    std::shared_ptr<Material> material;
    std::shared_ptr<MeshRenderer> mesh_renderer;
  };
  std::unordered_map<std::pair<std::string, int>, std::shared_ptr<Renderer>>
      _renderers;
  void update(const visualization_msgs::Marker &marker);

protected:
  MarkerDisplayBase();

public:
  virtual void renderSync(const RenderSyncContext &context) override;
  virtual void renderAsync(const RenderAsyncContext &context) override;
  // PROPERTY(std::shared_ptr<MaterialOverride>, materialOverride,
  //           std::make_shared<MaterialOverride>());
};
DECLARE_TYPE(MarkerDisplayBase, MeshDisplayBase);

class MarkerFrameDisplayBase : public GenericFrameDisplay<MarkerDisplayBase> {
public:
};

class MarkerDisplay : public MarkerDisplayBase {

public:
  PROPERTY(TopicProperty<visualization_msgs::Marker>, topic,
           "visualization_marker");
  MarkerDisplay();
};
DECLARE_TYPE(MarkerDisplay, MarkerDisplayBase);

class MarkerArrayDisplay : public MarkerDisplayBase {

public:
  PROPERTY(TopicProperty<visualization_msgs::MarkerArray>, topic,
           "visualization_marker_array");
  MarkerArrayDisplay();
};
DECLARE_TYPE(MarkerArrayDisplay, MarkerDisplayBase);
