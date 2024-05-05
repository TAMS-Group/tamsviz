// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include "renderwindow.h"

#include "../annotations/image.h"
#include "../core/topic.h"
#include "../core/tracks.h"
#include "../render/renderer.h"
#include "../render/texture.h"

struct ClampedVector2d : Eigen::Vector2d {
  template <class... T>
  ClampedVector2d(const T &...args) : Eigen::Vector2d(args...) {}
};
STRUCT_BEGIN(ClampedVector2d);
STRUCT_PROPERTY(x, min = 0, max = 1);
STRUCT_PROPERTY(y, min = 0, max = 1);
STRUCT_END();

AUTO_STRUCT_BEGIN(ImageWindowOptions);
AUTO_STRUCT_FIELD(bool, normalizeDepth, true);
AUTO_STRUCT_FIELD(bool, colorMapApply, false);
AUTO_STRUCT_FIELD(size_t, colorMapType, 9, min = 0);
AUTO_STRUCT_FIELD(double, brightness, 1);
AUTO_STRUCT_FIELD(double, saturation, 1);
AUTO_STRUCT_FIELD(bool, toneMapping, false);
AUTO_STRUCT_END();

class AnnotationViewBase;

class ImageWindow : public ContentWindowBase {
  // std::function<void(const ImageWindowOptions &)> _refresh_callback;
  std::shared_ptr<Subscriber<Message>> subscriber;
  static constexpr double minZoom() { return 0.01; }
  static constexpr double maxZoom() { return 1000000; }
  Watcher _options_watcher;

 public:
  std::shared_ptr<ImageAnnotationBase> new_annotation;
  std::shared_ptr<AnnotationSpan> new_annotation_span;
  std::shared_ptr<Type> annotation_type;
  std::unordered_map<std::shared_ptr<ImageAnnotationBase>, AnnotationViewBase *>
      annotation_views;
  ImageWindow();
  PROPERTY(std::string, topic, "");
  PROPERTY(ClampedVector2d, center, Eigen::Vector2d(0.5, 0.5));
  PROPERTY(double, zoom, 1.0, min = minZoom(), max = maxZoom());
  PROPERTY(ImageWindowOptions, options);
  virtual void refresh() override {
    LOG_DEBUG("refresh");
    ContentWindowBase::refresh();
    this->window()->update();
    // if (_options_watcher.changed(options())) {
    //   if (_refresh_callback) {
    //     _refresh_callback(options());
    //   }
    // }
  }
};
DECLARE_TYPE(ImageWindow, ContentWindowBase);
