// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#pragma once

#include "frame.h"

#include "../core/profiler.h"
#include "../core/topic.h"
#include "../core/watcher.h"
#include "../render/mesh.h"
#include "../scene/mesh.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

class PointCloudDisplayBase : public MeshDisplayBase {
  std::shared_ptr<MeshRenderer> _mesh_renderer;
  std::shared_ptr<Material> _material = std::make_shared<Material>();
  std::shared_ptr<MaterialRenderer> _material_renderer =
      std::make_shared<MaterialRenderer>(_material);

protected:
  PointCloudDisplayBase();
  void setPointCloud(const std::shared_ptr<const Message> &message);

public:
  PROPERTY(double, pointSize, 1.0, min = 1.0);
  PROPERTY(bool, pointColors, true);
};
DECLARE_TYPE(PointCloudDisplayBase, MeshDisplayBase);

class PointCloudDisplay : public PointCloudDisplayBase {
  Watcher _watcher;

public:
  PROPERTY(TopicProperty<sensor_msgs::PointCloud>, topic, "");
  virtual void renderSync(const RenderSyncContext &context) override;
};
DECLARE_TYPE_C(PointCloudDisplay, PointCloudDisplayBase, Sensor);

class PointCloud2Display : public PointCloudDisplayBase {
  Watcher _watcher;

public:
  PROPERTY(TopicProperty<sensor_msgs::PointCloud2>, topic, "");
  virtual void renderSync(const RenderSyncContext &context) override;
};
DECLARE_TYPE_C(PointCloud2Display, PointCloudDisplayBase, Sensor);
