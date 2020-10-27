// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "frame.h"

#include "../core/topic.h"
#include "../core/watcher.h"
#include "../render/mesh.h"
#include "../scene/mesh.h"

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>

#include <laser_geometry/laser_geometry.h>

class LaserScanDisplay : public MeshDisplayBase {
  std::shared_ptr<MeshRenderer> _mesh_renderer;
  std::shared_ptr<Material> _material = std::make_shared<Material>();
  laser_geometry::LaserProjection _projector;
  Watcher _watcher;

public:
  PROPERTY(double, pointRadius, 0.05, min = 0.0);
  PROPERTY(Color4, color, Color4(1, 1, 1, 1));
  PROPERTY(TopicProperty<sensor_msgs::LaserScan>, topic, "");
  virtual void renderSync(const RenderSyncContext &context) override;
  LaserScanDisplay() {}
};
DECLARE_TYPE(LaserScanDisplay, MeshDisplayBase);
