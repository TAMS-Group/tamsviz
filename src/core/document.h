// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "event.h"
#include "serialization.h"
#include "snapshot.h"

#include <Eigen/Dense>

STRUCT_BEGIN(Eigen::Vector2d);
STRUCT_PROPERTY(x);
STRUCT_PROPERTY(y);
STRUCT_END();

STRUCT_BEGIN(Eigen::Vector3d);
STRUCT_PROPERTY(x);
STRUCT_PROPERTY(y);
STRUCT_PROPERTY(z);
STRUCT_END();

struct Color4 {
  DECLARE_STRUCT_PROPERTY(double, r, 0.8);
  DECLARE_STRUCT_PROPERTY(double, g, 0.8);
  DECLARE_STRUCT_PROPERTY(double, b, 0.8);
  DECLARE_STRUCT_PROPERTY(double, a, 1.0);
  inline Color4() {}
  inline Color4(double r, double g, double b, double a = 1.0)
      : _r(r), _g(g), _b(b), _a(a) {}
  inline bool operator==(const Color4 &other) const {
    return r() == other.r() && g() == other.g() && b() == other.b() &&
           a() == other.a();
  }
  inline bool operator!=(const Color4 &other) const {
    return !(*this == other);
  }
  Eigen::Vector4f toLinearVector4f() const;
};
STRUCT_BEGIN(Color4);
STRUCT_PROPERTY(r, min = 0, max = 1);
STRUCT_PROPERTY(g, min = 0, max = 1);
STRUCT_PROPERTY(b, min = 0, max = 1);
STRUCT_PROPERTY(a, min = 0, max = 1);
STRUCT_END();

struct Color3 : Color4 {
  inline Color3() {}
  inline Color3(double r, double g, double b) : Color4(r, g, b) {}
};
STRUCT_BEGIN(Color3);
STRUCT_PROPERTY(r, min = 0, max = 1);
STRUCT_PROPERTY(g, min = 0, max = 1);
STRUCT_PROPERTY(b, min = 0, max = 1);
STRUCT_END();

struct Orientation {
  DECLARE_STRUCT_PROPERTY(double, yaw, 0.0);
  DECLARE_STRUCT_PROPERTY(double, pitch, 0.0);
  DECLARE_STRUCT_PROPERTY(double, roll, 0.0);
  inline bool operator==(const Orientation &other) const {
    return yaw() == other.yaw() && pitch() == other.pitch() &&
           roll() == other.roll();
  }
  inline bool operator!=(const Orientation &other) const {
    return !(*this == other);
  }
  Eigen::Isometry3d toIsometry3d() const;
};
STRUCT_BEGIN(Orientation);
STRUCT_PROPERTY(yaw, step_scale = 90);
STRUCT_PROPERTY(pitch, step_scale = 90);
STRUCT_PROPERTY(roll, step_scale = 90);
STRUCT_END();

struct Pose {
  DECLARE_STRUCT_PROPERTY(Eigen::Vector3d, position, Eigen::Vector3d::Zero());
  DECLARE_STRUCT_PROPERTY(Orientation, orientation, Orientation());
  inline bool operator==(const Pose &other) const {
    return position() == other.position() &&
           orientation() == other.orientation();
  }
  inline bool operator!=(const Pose &other) const { return !(*this == other); }
  Eigen::Isometry3d toIsometry3d() const;
};
STRUCT_BEGIN(Pose);
STRUCT_PROPERTY(position);
STRUCT_PROPERTY(orientation);
STRUCT_END();

struct SizeVector3d : Eigen::Vector3d {
  SizeVector3d() { setConstant(1.0); }
  SizeVector3d(double sx, double sy, double sz) : Eigen::Vector3d(sx, sy, sz) {}
};
STRUCT_BEGIN(SizeVector3d);
STRUCT_PROPERTY(x, min = 1e-6);
STRUCT_PROPERTY(y, min = 1e-6);
STRUCT_PROPERTY(z, min = 1e-6);
STRUCT_END();

struct SizeVector2d : Eigen::Vector2d {
  SizeVector2d() { setConstant(1.0); }
  SizeVector2d(double sx, double sy) : Eigen::Vector2d(sx, sy) {}
};
STRUCT_BEGIN(SizeVector2d);
STRUCT_PROPERTY(x, min = 1e-6);
STRUCT_PROPERTY(y, min = 1e-6);
STRUCT_END();

class RenderList;

struct RenderSyncContext {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  RenderList *render_list = nullptr;
};
struct RenderAsyncContext {
  RenderList *render_list = nullptr;
};

struct Display : Object {

protected:
  Display();
  virtual ~Display() {}

public:
  PROPERTY(std::string, name, "Display");
  template <class F>
  auto recurse(const F &f)
      -> decltype(f(std::shared_ptr<Display>(), std::shared_ptr<Display>())) {

    forEachObject(
        (void *)&f,
        [](void *context, const Object *parent, const Object *child) {
          auto p = parent ? std::dynamic_pointer_cast<Display>(
                                ((Object *)parent)->shared_from_this())
                          : nullptr;
          if (auto c = child ? std::dynamic_pointer_cast<Display>(
                                   ((Object *)child)->shared_from_this())
                             : nullptr) {
            (*(F *)context)(p, c);
          }
        },
        nullptr, this);
  }
  template <class F>
  auto recurse(const F &f) -> decltype(f(std::shared_ptr<Display>())) {
    forEachObject((void *)&f,
                  [](void *context, const Object *parent, const Object *child) {
                    auto *f = (F *)context;
                    if (auto c =
                            child ? std::dynamic_pointer_cast<Display>(
                                        ((Object *)child)->shared_from_this())
                                  : nullptr) {
                      (*(F *)context)(c);
                    }
                  },
                  nullptr, this);
  }
  virtual void renderSyncRecursive(const RenderSyncContext &context) {
    renderSync(context);
  }
  virtual void renderSync(const RenderSyncContext &context) {}
  virtual void renderAsync(const RenderAsyncContext &context) {}
};
DECLARE_TYPE(Display, Object);

struct DisplayGroupBase : Display {
protected:
  DisplayGroupBase() {}

public:
  PROPERTY(std::vector<std::shared_ptr<Display>>, displays,
           std::vector<std::shared_ptr<Display>>());
  virtual void renderSyncRecursive(const RenderSyncContext &context) override;
};
DECLARE_TYPE(DisplayGroupBase, Display);

struct Window : Object {
protected:
  Window() {}

public:
  PROPERTY(std::string, name, "Window");
};
DECLARE_TYPE(Window, Object);

class Transformer;
struct WorldDisplay : DisplayGroupBase {
private:
  static std::vector<std::string> _listFrames(const Property &);

public:
  std::shared_ptr<Transformer> transformer;
  PROPERTY(std::string, fixedFrame, "world", list = &WorldDisplay::_listFrames);
  PROPERTY(Color4, backgroundColor, Color4(0.3, 0.3, 0.3, 1.0));
  PROPERTY(double, ambientLighting, 1.0, min = 0.0, max = 1.0);
  virtual void renderSync(const RenderSyncContext &context) override;
  WorldDisplay();
};
DECLARE_TYPE(WorldDisplay, DisplayGroupBase);

struct AnnotationBase : Object {
protected:
  AnnotationBase() {}

public:
  PROPERTY(std::string, label, "");
};
DECLARE_TYPE(AnnotationBase, Object);

class QPainterPath;
struct ImageAnnotationBase : AnnotationBase {
protected:
  ImageAnnotationBase() {}

public:
  PROPERTY(std::vector<Eigen::Vector2d>, controlPoints);
  bool complete = false;
  std::shared_ptr<QPainterPath> shape;
  virtual void render() = 0;
  virtual void constrain() {}
};
DECLARE_TYPE(ImageAnnotationBase, AnnotationBase);

struct AnnotationSpan : Object {
  PROPERTY(std::string, label, "");
  PROPERTY(double, start, 0.0);
  PROPERTY(double, duration, 1.0);
  PROPERTY(std::vector<std::shared_ptr<AnnotationBase>>, annotations, {});
};
DECLARE_TYPE(AnnotationSpan, Object);

struct TrackBase : Object {
protected:
  TrackBase();

public:
  PROPERTY(double, color, 0.0, min = 0.0, max = 1.0, wrap = true);
  PROPERTY(std::string, label, "Label");
};
DECLARE_TYPE(TrackBase, Object);

struct AnnotationBranch : Object {
  PROPERTY(std::string, name);
  PROPERTY(std::vector<std::shared_ptr<AnnotationSpan>>, spans);
};
DECLARE_TYPE(AnnotationBranch, Object);

struct AnnotationTrack : TrackBase {
  PROPERTY(std::vector<std::shared_ptr<AnnotationBranch>>, branches);
  std::shared_ptr<AnnotationBranch> branch(bool create = false);
};
DECLARE_TYPE(AnnotationTrack, TrackBase);

struct GraphTrack : TrackBase {};
DECLARE_TYPE(GraphTrack, TrackBase);

struct Timeline : Object {
  PROPERTY(std::vector<std::shared_ptr<TrackBase>>, tracks);
};
DECLARE_TYPE(Timeline, Object);

struct Document : Object {
  std::string path;
  PROPERTY(std::shared_ptr<Timeline>, timeline, std::make_shared<Timeline>());
  PROPERTY(std::shared_ptr<Window>, window, nullptr);
  PROPERTY(std::shared_ptr<WorldDisplay>, display,
           std::make_shared<WorldDisplay>());
};
DECLARE_TYPE(Document, Object);
