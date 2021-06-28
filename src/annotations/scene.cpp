// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "scene.h"

#include "../core/interaction.h"
#include "../core/log.h"
#include "../core/workspace.h"
#include "../displays/interactive.h"
#include "../displays/shapes.h"
#include "../render/mesh.h"

#include <QColor>

SceneAnnotationBase::SceneAnnotationBase() {}

bool SceneAnnotationBase::selected() {
  return LockScope()->selection().contains(shared_from_this());
}

std::shared_ptr<SceneNode> SceneAnnotationBase::node() {
  if (_reset) {
    _reset = false;
    _node = nullptr;
  }
  if (!_node) {
    _node = std::make_shared<SceneNode>();
    update(_node);
    bool s = selected();
    std::weak_ptr<Object> me = shared_from_this();
    LockScope()->modified.connect(_node, [this, me, s]() {
      if (auto me__ = me.lock()) {
        bool v = selected();
        if (v != s) {
          LOG_DEBUG("selection " << int(s) << " " << int(v));
          _reset = true;
          GlobalEvents::instance()->redraw();
        }
      }
    });
  }
  return _node;
}

void SceneAnnotationBase::renderSync(
    const RenderSyncContext &context, const std::shared_ptr<TrackBase> &track,
    const std::shared_ptr<AnnotationSpan> &span) {
  if (!_context) {
    _context = std::make_shared<SceneContext>();
  }
  _context->nodes.clear();
  node()->renderSyncRecursive(context, *_context);
}

void SceneAnnotationBase::renderAsync(const RenderAsyncContext &context) {
  // LOG_DEBUG("render");
  if (_context) {
    for (auto &node : _context->nodes) {
      // LOG_DEBUG("async");
      node->renderAsync(context);
    }
  }
}

bool SceneAnnotationBase::pick(uint32_t id) const {
  if (_node) {
    return _node->pick(id);
  } else {
    return false;
  }
}

bool SceneAnnotationBase::interact(const Interaction &interaction) {
  if (_node) {
    bool rs = _node->interact(interaction);
    // LOG_DEBUG("SceneAnnotationBase::interact " << (int)rs);
    return rs;
  } else {
    return false;
  }
}

/*
void SceneAnnotationBase::clear() {
  LOG_ERROR("clear scene annotation");
  //std::lock_guard<std::mutex> lock(_mutex);
  //_node.reset();
  //_context.reset();
}
*/

struct InteractiveSceneAnnotationBase : SceneAnnotationBase {
private:
  std::shared_ptr<InteractiveMarkerArray> _markers;

protected:
  std::shared_ptr<SceneNode> _visual;
  std::shared_ptr<Material> _material;
  InteractiveSceneAnnotationBase() {}

public:
  PROPERTY(double, scale, 1.0, min = 0.0);
  // PROPERTY(Pose, transform);
  const std::shared_ptr<InteractiveMarkerArray> &
  markers(const std::shared_ptr<SceneNode> &node) {
    if (!_markers) {
      _markers = node->create<InteractiveMarkerArray>(
          std::make_shared<InteractiveMarkerParameters>());
    }
    return _markers;
  }
  virtual void update(const std::shared_ptr<SceneNode> &node) override {
    SceneAnnotationBase::update(node);
    _markers = nullptr;
  }
  virtual Eigen::Isometry3d virtualPose() const = 0;
  virtual void virtualPose(const Eigen::Isometry3d &pose) = 0;
  virtual void
  renderSync(const RenderSyncContext &context,
             const std::shared_ptr<TrackBase> &track,
             const std::shared_ptr<AnnotationSpan> &span) override {
    node();
    auto p = virtualPose();
    p.linear() *= scale();
    if (_visual) {
      _visual->pose(p);
    }
    if (_material) {
      auto color = QColor::fromHsvF(track->color(), 0.8, 0.8);
      _material->color() = Color3(color.redF(), color.greenF(), color.blueF());
    }
    if (_markers) {
      if (auto m = _markers->marker("")) {
        m->pose(p);
      }
    }
    SceneAnnotationBase::renderSync(context, track, span);
  }
  virtual bool interact(const Interaction &interaction) override {
    if (SceneAnnotationBase::interact(interaction)) {
      if (_markers) {
        if (auto m = _markers->marker("")) {
          if (interaction.finished) {
            ActionScope ws("Move handle");
            virtualPose(Eigen::Isometry3d(m->pose().matrix()));
            ws->modified();
          } else {
            LockScope ws;
            virtualPose(Eigen::Isometry3d(m->pose().matrix()));
            ws->modified();
          }
        }
      }
      return true;
    } else {
      return false;
    }
  }
};
DECLARE_TYPE(InteractiveSceneAnnotationBase, SceneAnnotationBase);

struct PointSceneAnnotation : InteractiveSceneAnnotationBase {
  PROPERTY(Eigen::Vector3d, position);
  virtual Eigen::Isometry3d virtualPose() const override {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = position();
    return pose;
  }
  virtual void virtualPose(const Eigen::Isometry3d &pose) override {
    position() = pose.translation();
  }
  // transform().toIsometry3d()
  // transform().fromIsometry3d
  virtual void update(const std::shared_ptr<SceneNode> &node) override {
    InteractiveSceneAnnotationBase::update(node);
    if (!_material) {
      _material = std::make_shared<Material>();
    }
    // if (!selected())
    {
      static auto mesh = std::make_shared<Mesh>(makeSphere().scale(0.22));
      _visual = node->create<MeshRenderer>(
          mesh, _material /*, [this](const Interaction &interaction) {
            position() +=
                (interaction.current.point - interaction.previous.point);
          }*/);
    }
    if (selected()) {
      visualization_msgs::InteractiveMarker marker;
      {
        visualization_msgs::InteractiveMarkerControl control;
        control.interaction_mode =
            visualization_msgs::InteractiveMarkerControl::MOVE_3D;
        control.always_visible = true;
        {
          visualization_msgs::Marker marker;
          marker.type = visualization_msgs::Marker::SPHERE;
          marker.scale.x = 0.45;
          marker.scale.y = 0.45;
          marker.scale.z = 0.45;
          marker.color.r = 0.5;
          marker.color.g = 0.5;
          marker.color.b = 0.5;
          marker.color.a = 0.2;
          control.markers.push_back(marker);
        }
        marker.controls.push_back(control);
      }
      {
        visualization_msgs::InteractiveMarkerControl control;
        control.orientation.w = 1;
        control.orientation.x = 1;
        control.orientation.y = 0;
        control.orientation.z = 0;
        control.name = "move_x";
        control.interaction_mode =
            visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        marker.controls.push_back(control);
      }
      {
        visualization_msgs::InteractiveMarkerControl control;
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 1;
        control.orientation.z = 0;
        control.name = "move_z";
        control.interaction_mode =
            visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        marker.controls.push_back(control);
      }
      {
        visualization_msgs::InteractiveMarkerControl control;
        control.orientation.w = 1;
        control.orientation.x = 0;
        control.orientation.y = 0;
        control.orientation.z = 1;
        control.name = "move_y";
        control.interaction_mode =
            visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
        marker.controls.push_back(control);
      }
      visualization_msgs::InteractiveMarkerInit init;
      init.markers.push_back(marker);
      markers(node)->init(init);
    }
  }
};
DECLARE_TYPE(PointSceneAnnotation, InteractiveSceneAnnotationBase);
