// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include "marker.h"

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#include <eigen_conversions/eigen_msg.h>

#include "../core/interaction.h"
#include "../core/workspace.h"
#include "axes.h"
#include "interactive.h"
#include "shapes.h"

class PointDisplayBase : public MeshDisplayBase {
  std::shared_ptr<MeshRenderer> _mesh_renderer;
  Watcher _mesh_watcher;

protected:
  PointDisplayBase() {}

public:
  PROPERTY(double, radius, 0.1, min = 0.0);
  PROPERTY(std::shared_ptr<Material>, material,
           std::make_shared<Material>(1, 1, 1));
  virtual void renderSync(const RenderSyncContext &context) override {
    if (_mesh_watcher.changed(radius())) {
      _mesh_renderer = node()->create<MeshRenderer>(
          std::make_shared<Mesh>(makeSphere(16, 8).scale(radius())),
          material());
    }
    MeshDisplayBase::renderSync(context);
  }
};
DECLARE_TYPE(PointDisplayBase, MeshDisplayBase);

class PointStampedDisplay : public PointDisplayBase {

public:
  PROPERTY(TopicProperty<geometry_msgs::PointStamped>, topic, "");
  virtual void renderSync(const RenderSyncContext &context) override {
    if (auto message = topic().message()) {
      node()->frame(message->header.frame_id);
      Eigen::Vector3d point;
      tf::pointMsgToEigen(message->point, point);
      Eigen::Isometry3d transform;
      transform.translation() = point;
      node()->pose(transform);
      node()->show();
    } else {
      node()->hide();
    }
    PointDisplayBase::renderSync(context);
  }
};
DECLARE_TYPE_C(PointStampedDisplay, PointDisplayBase, Geometry);

class PointStampedPublisherDisplay : public InteractivePoseDisplayBase {
  Publisher<geometry_msgs::PointStamped> _publisher;

public:
  PROPERTY(std::string, topic);
  PointStampedPublisherDisplay() {
    visualization_msgs::InteractiveMarkerInit init;
    init.markers.push_back(makePointMarker());
    _markers->init(init);
  }
  virtual void publish(const std::string &frame,
                       const Eigen::Isometry3d &pose) override {
    if (_publish_watcher.changed(topic(), frame, poseToArray(pose),
                                 publisherClock())) {
      geometry_msgs::PointStamped m;
      tf::pointEigenToMsg(pose.translation(), m.point);
      m.header.frame_id = frame;
      _publisher.publish(topic(), m);
    }
  }
};
DECLARE_TYPE_C(PointStampedPublisherDisplay, InteractivePoseDisplayBase,
               Geometry);

class PointDisplay : public PointDisplayBase {

public:
  PROPERTY(Frame, frame);
  PROPERTY(TopicProperty<geometry_msgs::Point>, topic, "");
  virtual void renderSync(const RenderSyncContext &context) override {
    node()->frame(frame());
    if (auto message = topic().message()) {
      Eigen::Vector3d point;
      tf::pointMsgToEigen(*message, point);
      Eigen::Isometry3d transform;
      transform.translation() = point;
      node()->pose(transform);
      node()->show();
    } else {
      node()->hide();
    }
    PointDisplayBase::renderSync(context);
  }
};
DECLARE_TYPE_C(PointDisplay, PointDisplayBase, Geometry);

class PointPublisherDisplay : public InteractivePoseDisplayBase {
  Publisher<geometry_msgs::Point> _publisher;

public:
  PROPERTY(std::string, topic);
  PointPublisherDisplay() {
    visualization_msgs::InteractiveMarkerInit init;
    init.markers.push_back(makePointMarker());
    _markers->init(init);
  }
  virtual void publish(const std::string &frame,
                       const Eigen::Isometry3d &pose) override {
    if (_publish_watcher.changed(topic(), frame, poseToArray(pose),
                                 publisherClock())) {
      geometry_msgs::Point m;
      tf::pointEigenToMsg(pose.translation(), m);
      _publisher.publish(topic(), m);
    }
  }
};
DECLARE_TYPE_C(PointPublisherDisplay, InteractivePoseDisplayBase, Geometry);

class PoseDisplayBase : public MeshDisplayBase {
  std::shared_ptr<MeshRenderer> _mesh_renderer;
  Watcher _mesh_watcher;

protected:
  PoseDisplayBase() {}

public:
  PROPERTY(double, radius, 0.1, min = 0.0);
  PROPERTY(double, length, 1.0, min = 0.0);
  PROPERTY(std::shared_ptr<Material>, material,
           std::make_shared<Material>(1, 1, 1));
  virtual void renderSync(const RenderSyncContext &context) override {
    if (_mesh_watcher.changed(length(), radius())) {
      _mesh_renderer = node()->create<MeshRenderer>(
          std::make_shared<Mesh>(makeAxes(length(), radius(), 24)), material());
    }
    MeshDisplayBase::renderSync(context);
  }
};
DECLARE_TYPE(PoseDisplayBase, MeshDisplayBase);

class PoseDisplay : public PoseDisplayBase {
public:
  PROPERTY(Frame, frame);
  PROPERTY(TopicProperty<geometry_msgs::Pose>, topic, "");
  virtual void renderSync(const RenderSyncContext &context) override {
    node()->frame(frame());
    if (auto message = topic().message()) {
      Eigen::Isometry3d transform;
      tf::poseMsgToEigen(*message, transform);
      node()->pose(transform);
      node()->show();
    } else {
      node()->hide();
    }
    PoseDisplayBase::renderSync(context);
  }
};
DECLARE_TYPE_C(PoseDisplay, PoseDisplayBase, Geometry);

class PosePublisherDisplay : public InteractivePoseDisplayBase {
  Publisher<geometry_msgs::Pose> _publisher;

public:
  PROPERTY(std::string, topic);
  PosePublisherDisplay() {
    visualization_msgs::InteractiveMarkerInit init;
    init.markers.push_back(makePoseMarker());
    _markers->init(init);
  }
  virtual void publish(const std::string &frame,
                       const Eigen::Isometry3d &pose) override {
    if (_publish_watcher.changed(topic(), frame, poseToArray(pose),
                                 publisherClock())) {
      geometry_msgs::Pose m;
      tf::poseEigenToMsg(pose, m);
      _publisher.publish(topic(), m);
    }
  }
};
DECLARE_TYPE_C(PosePublisherDisplay, InteractivePoseDisplayBase, Geometry);

class PoseStampedDisplay : public PoseDisplayBase {
public:
  PROPERTY(TopicProperty<geometry_msgs::PoseStamped>, topic, "");
  virtual void renderSync(const RenderSyncContext &context) override {
    if (auto message = topic().message()) {
      node()->frame(message->header.frame_id);
      Eigen::Isometry3d transform;
      tf::poseMsgToEigen(message->pose, transform);
      node()->pose(transform);
      node()->show();
    } else {
      node()->hide();
    }
    PoseDisplayBase::renderSync(context);
  }
};
DECLARE_TYPE_C(PoseStampedDisplay, PoseDisplayBase, Geometry);

class PoseStampedPublisherDisplay : public InteractivePoseDisplayBase {
  Publisher<geometry_msgs::PoseStamped> _publisher;

public:
  PROPERTY(std::string, topic);
  PoseStampedPublisherDisplay() {
    visualization_msgs::InteractiveMarkerInit init;
    init.markers.push_back(makePoseMarker());
    _markers->init(init);
  }
  virtual void publish(const std::string &frame,
                       const Eigen::Isometry3d &pose) override {
    if (_publish_watcher.changed(topic(), frame, poseToArray(pose),
                                 publisherClock())) {
      geometry_msgs::PoseStamped m;
      tf::poseEigenToMsg(pose, m.pose);
      m.header.frame_id = frame;
      _publisher.publish(topic(), m);
    }
  }
};
DECLARE_TYPE_C(PoseStampedPublisherDisplay, InteractivePoseDisplayBase,
               Geometry);

class QuaternionDisplay : public PoseDisplayBase {
public:
  PROPERTY(Frame, frame);
  PROPERTY(TopicProperty<geometry_msgs::Quaternion>, topic, "");
  virtual void renderSync(const RenderSyncContext &context) override {
    node()->frame(frame());
    if (auto message = topic().message()) {
      Eigen::Quaterniond quat;
      tf::quaternionMsgToEigen(*message, quat);
      node()->pose(Eigen::Isometry3d(quat));
      node()->show();
    } else {
      node()->hide();
    }
    PoseDisplayBase::renderSync(context);
  }
};
DECLARE_TYPE_C(QuaternionDisplay, PoseDisplayBase, Geometry);

class QuaternionPublisherDisplay : public InteractivePoseDisplayBase {
  Publisher<geometry_msgs::Quaternion> _publisher;

public:
  PROPERTY(std::string, topic);
  QuaternionPublisherDisplay() {
    visualization_msgs::InteractiveMarkerInit init;
    init.markers.push_back(makeRotationMarker());
    _markers->init(init);
  }
  virtual void publish(const std::string &frame,
                       const Eigen::Isometry3d &pose) override {
    if (_publish_watcher.changed(topic(), frame, poseToArray(pose),
                                 publisherClock())) {
      geometry_msgs::Quaternion m;
      tf::quaternionEigenToMsg(Eigen::Quaterniond(pose.linear()), m);
      _publisher.publish(topic(), m);
    }
  }
};
DECLARE_TYPE_C(QuaternionPublisherDisplay, InteractivePoseDisplayBase,
               Geometry);

class QuaternionStampedDisplay : public PoseDisplayBase {
public:
  PROPERTY(TopicProperty<geometry_msgs::QuaternionStamped>, topic, "");
  virtual void renderSync(const RenderSyncContext &context) override {
    if (auto message = topic().message()) {
      Eigen::Quaterniond quat;
      tf::quaternionMsgToEigen(message->quaternion, quat);
      node()->pose(Eigen::Isometry3d(quat));
      node()->show();
    } else {
      node()->hide();
    }
    PoseDisplayBase::renderSync(context);
  }
};
DECLARE_TYPE_C(QuaternionStampedDisplay, PoseDisplayBase, Geometry);

class QuaternionStampedPublisherDisplay : public InteractivePoseDisplayBase {
  Publisher<geometry_msgs::QuaternionStamped> _publisher;

public:
  PROPERTY(std::string, topic);
  QuaternionStampedPublisherDisplay() {
    visualization_msgs::InteractiveMarkerInit init;
    init.markers.push_back(makeRotationMarker());
    _markers->init(init);
  }
  virtual void publish(const std::string &frame,
                       const Eigen::Isometry3d &pose) override {
    if (_publish_watcher.changed(topic(), frame, poseToArray(pose),
                                 publisherClock())) {
      geometry_msgs::QuaternionStamped m;
      tf::quaternionEigenToMsg(Eigen::Quaterniond(pose.linear()), m.quaternion);
      m.header.frame_id = frame;
      _publisher.publish(topic(), m);
    }
  }
};
DECLARE_TYPE_C(QuaternionStampedPublisherDisplay, InteractivePoseDisplayBase,
               Geometry);

class GeometryPublisherDisplayBase : public MeshDisplayBase {
  struct Data {
    std::mutex mutex;
    std::condition_variable condition;
    std::weak_ptr<GeometryPublisherDisplayBase> display;
    volatile bool exit_flag = false;
  };
  std::shared_ptr<Data> _data;

public:
  ~GeometryPublisherDisplayBase() {
    std::unique_lock<std::mutex> lock(_data->mutex);
    _data->exit_flag = true;
    _data->condition.notify_all();
  }
  virtual void refresh() override {
    if (!_data) {
      auto data = _data = std::make_shared<Data>();
      data->display = std::dynamic_pointer_cast<GeometryPublisherDisplayBase>(
          shared_from_this());
      std::thread([data]() {
        LOG_DEBUG("start publisher thread");
        auto t = std::chrono::steady_clock::now();
        while (true) {
          t += std::chrono::seconds(1);
          {
            std::unique_lock<std::mutex> lock(data->mutex);
            while (true) {
              if (data->exit_flag) {
                LOG_DEBUG("exit publisher thread");
                return;
              }
              if (std::chrono::steady_clock::now() >= t) {
                break;
              }
              data->condition.wait_until(lock, t);
            }
          }
          if (auto me = data->display.lock()) {
            LockScope ws;
            me->publish();
          }
        }
      }).detach();
    }
    publish();
  }
  virtual void publish() = 0;
};
DECLARE_TYPE(GeometryPublisherDisplayBase, MeshDisplayBase);

class Vector3Node : public SceneNode {
  std::shared_ptr<MeshRenderer> _line_renderer, _tip_renderer;

public:
  Vector3Node(const std::shared_ptr<Material> &material) {
    _line_renderer = create<MeshRenderer>(
        std::make_shared<Mesh>(
            makeCylinder(16).scale(1, 1, 0.5).translate(0, 0, 0.5)),
        material);
    _tip_renderer = create<MeshRenderer>(
        std::make_shared<Mesh>(makeCone(16).scale(1, 1, 1).translate(0, 0, 0)),
        material);
  }
  void setVector(const Eigen::Vector3d &vector, double arrow_length,
                 double arrow_radius) {
    double length = arrow_length * vector.norm();
    double tip_length =
        std::max(1e-6, std::min(length * 0.5, arrow_radius * 4));
    double tip_radius = tip_length * 0.5;
    double line_length = std::max(1e-6, length - tip_length);
    double line_radius = tip_length * 0.25;
    Eigen::Vector3d z = vector.normalized();
    Eigen::Vector3d y = z.unitOrthogonal();
    Eigen::Vector3d x = y.cross(z).normalized();
    {
      Eigen::Affine3d transform;
      transform.setIdentity();
      transform.linear().col(0) = x * line_radius;
      transform.linear().col(1) = y * line_radius;
      transform.linear().col(2) = z * line_length;
      _line_renderer->pose(transform);
    }
    {
      Eigen::Affine3d transform;
      transform.setIdentity();
      transform.linear().col(0) = x * tip_radius;
      transform.linear().col(1) = y * tip_radius;
      transform.linear().col(2) = z * tip_length;
      transform.translation() = z * line_length;
      _tip_renderer->pose(transform);
    }
  }
};

class Vector3DisplayBase : public MeshDisplayBase {
  std::shared_ptr<Vector3Node> _node;

protected:
  void _setVector(const Eigen::Vector3d &vector) {
    _node->setVector(vector, length(), radius());
  }
  Vector3DisplayBase() {}

public:
  PROPERTY(double, radius, 0.1, min = 0.0);
  PROPERTY(double, length, 1.0, min = 0.0);
  PROPERTY(std::shared_ptr<Material>, material,
           std::make_shared<Material>(1, 1, 1));
  virtual void refresh() override {
    if (!_node) {
      _node = node()->create<Vector3Node>(material());
      MeshDisplayBase::refresh();
    }
  }
};
DECLARE_TYPE(Vector3DisplayBase, MeshDisplayBase);

class Vector3Display : public Vector3DisplayBase {

public:
  PROPERTY(Frame, frame);
  PROPERTY(TopicProperty<geometry_msgs::Vector3>, topic, "");
  virtual void renderSync(const RenderSyncContext &context) override {
    node()->frame(frame());
    if (auto message = topic().message()) {
      Eigen::Vector3d vector;
      tf::vectorMsgToEigen(*message, vector);
      _setVector(vector);
      node()->show();
    } else {
      node()->hide();
    }
    Vector3DisplayBase::renderSync(context);
  }
};
DECLARE_TYPE_C(Vector3Display, Vector3DisplayBase, Geometry);

class Vector3PublisherDisplayBase : public GeometryPublisherDisplayBase {
  std::shared_ptr<Vector3Node> _node;
  Watcher _watcher;
  double _interaction_factor = 1;

public:
  PROPERTY(Frame, frame);
  PROPERTY(Eigen::Vector3d, vector);
  PROPERTY(double, radius, 0.1, min = 0.0);
  PROPERTY(double, length, 1.0, min = 0.0);
  PROPERTY(std::shared_ptr<Material>, material,
           std::make_shared<Material>(1, 1, 1));
  virtual void renderSync(const RenderSyncContext &context) override {
    if (_watcher.changed(vector().x(), vector().y(), vector().z())) {
      publish();
    }
    _node->setVector(vector(), length(), radius());
    _node->frame(frame());
    GeometryPublisherDisplayBase::renderSync(context);
  }
  virtual void refresh() override {
    if (!_node) {
      _node = node()->create<Vector3Node>(material());
      GeometryPublisherDisplayBase::refresh();
    }
  }
  virtual bool interact(const Interaction &interaction) override {
    if (GeometryPublisherDisplayBase::interact(interaction)) {
      return true;
    }
    Eigen::Affine3d inverse_pose = _node->framePose().inverse();
    if (interaction.pressed) {
      _interaction_factor =
          1.0 /
          std::max(1e-3,
                   std::min(1.0, vector().normalized().dot(
                                     inverse_pose * interaction.begin.point) /
                                     (vector().norm() * length()))) /
          length();
    }
    vector() += (inverse_pose * interaction.current.point -
                 inverse_pose * interaction.previous.point) *
                _interaction_factor;
    return true;
  }
};
DECLARE_TYPE(Vector3PublisherDisplayBase, GeometryPublisherDisplayBase);

class Vector3PublisherDisplay : public Vector3PublisherDisplayBase {
  Publisher<geometry_msgs::Vector3> _publisher;

public:
  PROPERTY(std::string, topic);
  virtual void publish() override {
    Eigen::Vector3d v = vector();
    geometry_msgs::Vector3 m;
    tf::vectorEigenToMsg(v, m);
    _publisher.publish(topic(), m);
  }
};
DECLARE_TYPE_C(Vector3PublisherDisplay, Vector3PublisherDisplayBase, Geometry);

class Vector3StampedPublisherDisplay : public Vector3PublisherDisplayBase {
  Publisher<geometry_msgs::Vector3Stamped> _publisher;

public:
  PROPERTY(std::string, topic);
  virtual void publish() override {
    Eigen::Vector3d v = vector();
    geometry_msgs::Vector3Stamped m;
    tf::vectorEigenToMsg(v, m.vector);
    m.header.frame_id = frame().name();
    _publisher.publish(topic(), m);
  }
};
DECLARE_TYPE_C(Vector3StampedPublisherDisplay, Vector3PublisherDisplayBase,
               Geometry);

class Vector3StampedDisplay : public Vector3DisplayBase {

public:
  PROPERTY(TopicProperty<geometry_msgs::Vector3Stamped>, topic);
  virtual void renderSync(const RenderSyncContext &context) override {
    if (auto message = topic().message()) {
      node()->frame(message->header.frame_id);
      Eigen::Vector3d vector;
      tf::vectorMsgToEigen(message->vector, vector);
      _setVector(vector);
      node()->show();
    } else {
      node()->hide();
    }
    Vector3DisplayBase::renderSync(context);
  }
};
DECLARE_TYPE_C(Vector3StampedDisplay, Vector3DisplayBase, Geometry);
