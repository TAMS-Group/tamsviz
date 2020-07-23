// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "renderwindow.h"

#include "../annotations/image.h"
#include "../core/topic.h"
#include "../render/renderer.h"
#include "../render/texture.h"

struct ClampedVector2d : Eigen::Vector2d {
  template <class... T>
  ClampedVector2d(const T &... args) : Eigen::Vector2d(args...) {}
};
STRUCT_BEGIN(ClampedVector2d);
STRUCT_PROPERTY(x, min = 0, max = 1);
STRUCT_PROPERTY(y, min = 0, max = 1);
STRUCT_END();

class AnnotationView;

class ImageWindow : public ContentWindowBase {
  std::shared_ptr<Subscriber<Message>> subscriber;
  static constexpr double minZoom() { return 0.01; }
  static constexpr double maxZoom() { return 1000000; }

public:
  std::shared_ptr<ImageAnnotationBase> new_annotation;
  std::shared_ptr<AnnotationSpan> new_annotation_span;
  std::shared_ptr<Type> annotation_type;
  std::unordered_map<std::shared_ptr<ImageAnnotationBase>, AnnotationView *>
      annotation_views;
  ImageWindow();
  PROPERTY(std::string, topic, "");
  PROPERTY(ClampedVector2d, center, Eigen::Vector2d(0.5, 0.5));
  PROPERTY(double, zoom, 1.0, min = minZoom(), max = maxZoom());
};
DECLARE_TYPE(ImageWindow, ContentWindowBase);
