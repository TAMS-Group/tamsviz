// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "interactive.h"

#include "../core/interaction.h"
#include "../core/profiler.h"
#include "../core/workspace.h"

#include <eigen_conversions/eigen_msg.h>

#include <interactive_markers/tools.h>

InteractiveMarkerControl::InteractiveMarkerControl(
    const visualization_msgs::InteractiveMarkerControl &message,
    const std::shared_ptr<InteractiveMarkerParameters> &params,
    InteractiveMarker *parent)
    : _parent(parent), _params(params) {
  update(message);
}

void InteractiveMarkerControl::update(
    const visualization_msgs::InteractiveMarkerControl &message) {
  std::lock_guard<std::mutex> lock(_mutex);
  PROFILER();
  tf::quaternionMsgToEigen(message.orientation, _orientation);
  if (_orientation.norm() == 0) {
    _orientation = Eigen::Quaterniond::Identity();
  } else {
    _orientation.normalize();
  }
  _name = message.name;
  _interaction_mode = message.interaction_mode;
  _markers.resize(message.markers.size());
  for (size_t i = 0; i < message.markers.size(); i++) {
    if (!_markers.at(i)) {
      _markers.at(i) = create<VisualizationMarker>(message.markers[i]);
    } else {
      _markers.at(i)->update(message.markers[i]);
    }
  }
}

bool InteractiveMarkerControl::interact(const Interaction &interaction) {
  std::lock_guard<std::mutex> lock(_mutex);
  PROFILER();
  if (pick(interaction.id)) {
    LOG_DEBUG("interactive marker interaction mode " << _interaction_mode);
    if (_interaction_mode != 0) {
      Eigen::Affine3d p = _parent->pose();
      Eigen::Affine3d control_pose =
          _parent->renderPose() * Eigen::AngleAxisd(_orientation);
      Eigen::Matrix3d control_orientation = control_pose.linear();
      switch (_interaction_mode) {
      case visualization_msgs::InteractiveMarkerControl::MOVE_3D: {
        p.translation() +=
            _parent->framePose().linear().inverse() *
            (interaction.current.point - interaction.previous.point);
        break;
      }
      case visualization_msgs::InteractiveMarkerControl::MOVE_PLANE: {
        p.translation() += _parent->framePose().linear().inverse() *
                           interaction.projectPlane(control_orientation *
                                                    Eigen::Vector3d::UnitX());
        break;
      }
      case visualization_msgs::InteractiveMarkerControl::MOVE_AXIS: {
        p.translation() += _parent->framePose().linear().inverse() *
                           interaction.projectToAxis(control_orientation *
                                                     Eigen::Vector3d::UnitX());
        break;
      }
      case visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS:
      case visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE: {
        Eigen::Vector3d a = control_pose.inverse() *
                            interaction.previous.projectPlane(
                                control_pose.translation(),
                                control_orientation * Eigen::Vector3d::UnitX());
        Eigen::Vector3d b = control_pose.inverse() *
                            interaction.current.projectPlane(
                                control_pose.translation(),
                                control_orientation * Eigen::Vector3d::UnitX());
        Eigen::Matrix3d rotation =
            Eigen::AngleAxisd(
                std::atan2(a.y(), a.z()) - std::atan2(b.y(), b.z()),
                Eigen::AngleAxisd(_orientation) * Eigen::Vector3d::UnitX())
                .matrix();
        p.linear() = (p.linear() * rotation).eval();
        if (_interaction_mode ==
            visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE) {
          p.translation() +=
              _parent->framePose().linear().inverse() *
              (control_orientation * (b.normalized() * (b.norm() - a.norm())));
        }
        break;
      }
      case visualization_msgs::InteractiveMarkerControl::ROTATE_3D:
      case visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D: {
        Eigen::Vector3d a =
            _parent->renderPose().inverse() * interaction.previous.point;
        Eigen::Vector3d b =
            _parent->renderPose().inverse() * interaction.current.point;
        Eigen::Quaterniond quat = Eigen::Quaterniond::Identity();
        quat.setFromTwoVectors(a, b);
        p *= Eigen::AngleAxisd(quat);
        if (_interaction_mode ==
            visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D) {
          p.translation() += _parent->framePose().linear().inverse() *
                             (_parent->renderPose().linear() *
                              (b.normalized() * (b.norm() - a.norm())));
        }
        break;
      }
      }
      _parent->pose(p);
      {
        visualization_msgs::InteractiveMarkerFeedback feedback;
        feedback.marker_name = parentInteractiveMarker()->markerName();
        feedback.control_name = controlName();
        if (interaction.pressed) {
          feedback.event_type =
              visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN;
        } else if (interaction.finished) {
          feedback.event_type =
              visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP;
        } else {
          feedback.event_type =
              visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE;
        }
        feedback.header.frame_id = _parent->frame().name();
        feedback.client_id = ros::this_node::getNamespace();
        tf::pointEigenToMsg(_parent->framePose().inverse() *
                                interaction.current.point,
                            feedback.mouse_point);
        feedback.mouse_point_valid = true;
        tf::poseEigenToMsg(_parent->pose(), feedback.pose);
        parentInteractiveMarker()->parentInteractiveMarkerArray()->feedback(
            feedback);
      }
      return true;
    }
  }
  return false;
}

InteractiveMarker::InteractiveMarker(
    const visualization_msgs::InteractiveMarker &message,
    const std::shared_ptr<InteractiveMarkerParameters> &params,
    InteractiveMarkerArray *parent)
    : _parent(parent), _params(params) {
  _text = create<TextRenderer>(_params->description_material);
  update(message);
}

void InteractiveMarker::update(const visualization_msgs::InteractiveMarker &m) {
  PROFILER();
  auto marker = m;
  interactive_markers::autoComplete(marker, false);
  std::lock_guard<std::mutex> lock(_mutex);
  if (_dragged) {
    return;
  }
  _scale = m.scale;
  _text->text(marker.description);
  {
    Eigen::Affine3d p = Eigen::Affine3d::Identity();
    tf::poseMsgToEigen(marker.pose, p);
    pose(p);
  }
  _name = m.name;
  frame(marker.header.frame_id);
  _controls.resize(marker.controls.size());
  for (size_t i = 0; i < marker.controls.size(); i++) {
    auto &m = _controls.at(i);
    if (!m) {
      // LOG_DEBUG("creating interactive marker control");
      m = create<InteractiveMarkerControl>(marker.controls.at(i), _params,
                                           this);
    } else {
      m->update(marker.controls.at(i));
    }
  }
}

void InteractiveMarker::renderSync(const RenderSyncContext &context) {
  SceneNode::renderSync(context);
  _text->size(_scale * _params->description_size);
  _text->offset(_params->description_offset * _scale);
  _text->visible(_params->show_descriptions && _params->description_size > 0);
}

bool InteractiveMarker::interact(const Interaction &interaction) {
  std::lock_guard<std::mutex> lock(_mutex);
  PROFILER();
  bool x = SceneNode::interact(interaction);
  if (x) {
    _dragged = !interaction.finished;
  }
  return x;
}

void InteractiveMarker::update(
    const visualization_msgs::InteractiveMarkerPose &message) {
  std::lock_guard<std::mutex> lock(_mutex);
  PROFILER();
  if (_dragged) {
    return;
  }
  frame(message.header.frame_id);
  Eigen::Affine3d p = Eigen::Affine3d::Identity();
  tf::poseMsgToEigen(message.pose, p);
  pose(p);
}

InteractiveMarkerArray::InteractiveMarkerArray(
    const std::shared_ptr<InteractiveMarkerParameters> &params)
    : _params(params) {}

void InteractiveMarkerArray::renderSync(const RenderSyncContext &context) {
  SceneNode::renderSync(context);
}

void InteractiveMarkerArray::init(
    const visualization_msgs::InteractiveMarkerInit &init) {
  std::lock_guard<std::mutex> lock(_mutex);
  PROFILER();
  auto previous_markers = _markers;
  _markers.clear();
  for (auto &marker : init.markers) {
    auto &m = _markers[marker.name];
    m = previous_markers[marker.name];
    if (!m) {
      m = create<InteractiveMarker>(marker, _params, this);
    } else {
      m->update(marker);
    }
  }
}

void InteractiveMarkerArray::update(
    const visualization_msgs::InteractiveMarkerUpdate &update) {
  std::lock_guard<std::mutex> lock(_mutex);
  PROFILER();
  for (auto &erase : update.erases) {
    _markers.erase(erase);
  }
  for (auto &pose : update.poses) {
    auto iter = _markers.find(pose.name);
    if (iter != _markers.end()) {
      iter->second->update(pose);
    }
  }
  for (auto &marker : update.markers) {
    auto &m = _markers[marker.name];
    if (!m) {
      m = create<InteractiveMarker>(marker, _params, this);
    } else {
      m->update(marker);
    }
  }
}

std::shared_ptr<InteractiveMarker>
InteractiveMarkerArray::marker(const std::string &name) {
  auto iter = _markers.find(name);
  if (iter != _markers.end()) {
    return iter->second;
  } else {
    return nullptr;
  }
}

InteractiveMarkerDisplayBase::InteractiveMarkerDisplayBase() {
  _params = std::make_shared<InteractiveMarkerParameters>();
  _params->description_material = std::make_shared<Material>();
  _markers = node()->create<InteractiveMarkerArray>(_params);
}

void InteractiveMarkerDisplayBase::renderSync(
    const RenderSyncContext &context) {
  MeshDisplayBase::renderSync(context);
}

InteractiveMarkerDisplay::InteractiveMarkerDisplay() {
  _publisher = std::make_shared<
      Publisher<visualization_msgs::InteractiveMarkerFeedback>>();
  auto *pub_ptr = _publisher.get();
  _markers->feedback.connect(
      _publisher,
      [pub_ptr](const visualization_msgs::InteractiveMarkerFeedback &feedback) {
        pub_ptr->publish(feedback);
      });
}

void InteractiveMarkerDisplay::renderSync(const RenderSyncContext &context) {
  _params->show_descriptions = showDescriptions();
  _params->description_size = descriptionSize();
  _params->description_offset = descriptionOffset();
  _params->description_material->color(descriptionColor());
  _params->description_material->opacity(descriptionOpacity());
  if (_watcher.changed(topicNamespace())) {
    auto markers = _markers;
    _update_subscriber = std::make_shared<
        Subscriber<visualization_msgs::InteractiveMarkerUpdate>>(
        topicNamespace() + "/update", markers,
        [markers](const std::shared_ptr<
                  const visualization_msgs::InteractiveMarkerUpdate> &update) {
          if (update) {
            // LOG_DEBUG("interactive marker update");
            markers->update(*update);
          }
        });
    _init_subscriber =
        std::make_shared<Subscriber<visualization_msgs::InteractiveMarkerInit>>(
            topicNamespace() + "/update_full", markers,
            [markers](const std::shared_ptr<
                      const visualization_msgs::InteractiveMarkerInit> &init) {
              if (init) {
                // LOG_DEBUG("interactive marker init");
                markers->init(*init);
              }
            });
    _publisher->topic(topicNamespace() + "/feedback");
  }
  InteractiveMarkerDisplayBase::renderSync(context);
}

std::vector<std::string> InteractiveMarkerDisplay::listTopicNamespaces() {
  std::string postfix = "/update";
  std::vector<std::string> all = TopicManager::instance()->listTopics(
      ros::message_traits::DataType<
          visualization_msgs::InteractiveMarkerUpdate>::value());
  std::vector<std::string> ret;
  for (auto &n : all) {
    if (n.size() > postfix.size()) {
      if (n.substr(n.size() - postfix.size()) == postfix) {
        ret.push_back(n.substr(0, n.size() - postfix.size()));
      }
    }
  }
  return ret;
}

InteractivePoseDisplayBase::InteractivePoseDisplayBase() {}

void InteractivePoseDisplayBase::renderSync(const RenderSyncContext &context) {
  if (auto m = _markers->marker("")) {
    m->frame(frame());
    {
      auto p = transform().toIsometry3d();
      p.linear() *= scale();
      m->pose(p);
    }
  }
  InteractiveMarkerDisplayBase::renderSync(context);
  if (auto m = _markers->marker("")) {
    publish(frame().empty()
                ? LockScope()->document()->display()->transformer->root()
                : frame().name(),
            Eigen::Isometry3d(m->pose().matrix()));
  }
}

bool InteractivePoseDisplayBase::interact(const Interaction &interaction) {
  if (InteractiveMarkerDisplayBase::interact(interaction)) {
    if (auto m = _markers->marker("")) {
      if (interaction.finished) {
        ActionScope ws("Move handle");
        transform().fromIsometry3d(Eigen::Isometry3d(m->pose().matrix()));
        ws->modified();
      } else {
        LockScope ws;
        transform().fromIsometry3d(Eigen::Isometry3d(m->pose().matrix()));
        ws->modified();
      }
    }
    return true;
  } else {
    return false;
  }
}

PointPublisherDisplay::PointPublisherDisplay() {
  visualization_msgs::InteractiveMarker marker;
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_3D;
    control.always_visible = true;
    {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.scale.x = 0.45;
      marker.scale.y = 0.45;
      marker.scale.z = 0.45;
      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 0.5;
      marker.color.a = 1.0;
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
  _markers->init(init);
}

void PointPublisherDisplay::publish(const std::string &frame,
                                    const Eigen::Isometry3d &pose) {
  geometry_msgs::PointStamped m;
  tf::pointEigenToMsg(pose.translation(), m.point);
  m.header.frame_id = frame;
  _publisher.publish(topic(), m);
}

PosePublisherDisplay::PosePublisherDisplay() {
  visualization_msgs::InteractiveMarker marker;
  {
    visualization_msgs::InteractiveMarkerControl control;
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_3D;
    control.always_visible = true;
    {
      visualization_msgs::Marker marker;
      marker.type = visualization_msgs::Marker::CUBE;
      marker.scale.x = 0.45;
      marker.scale.y = 0.45;
      marker.scale.z = 0.45;
      marker.color.r = 0.5;
      marker.color.g = 0.5;
      marker.color.b = 0.5;
      marker.color.a = 1.0;
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
    control.name = "rotate_x";
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
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
    control.name = "rotate_z";
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
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
    control.name = "rotate_y";
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode =
        visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    marker.controls.push_back(control);
  }
  visualization_msgs::InteractiveMarkerInit init;
  init.markers.push_back(marker);
  _markers->init(init);
}

void PosePublisherDisplay::publish(const std::string &frame,
                                   const Eigen::Isometry3d &pose) {
  geometry_msgs::PoseStamped m;
  tf::poseEigenToMsg(pose, m.pose);
  m.header.frame_id = frame;
  _publisher.publish(topic(), m);
}
