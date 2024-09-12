// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#pragma once

#include "../core/document.h"

class QPainterPath;
class QPainter;
class QColor;

struct ImageAnnotationBase : AnnotationBase {
 protected:
  ImageAnnotationBase() {}

 public:
  PROPERTY(std::vector<Eigen::Vector2d>, controlPoints);
  PROPERTY(std::string, topic);
  bool complete = false;
  std::shared_ptr<QPainterPath> shape;
  virtual void render() = 0;
  virtual void heatmap(QPainter& painter, const QColor& color);
  virtual void constrain() {}
};
DECLARE_TYPE(ImageAnnotationBase, AnnotationBase);

struct PointImageAnnotation : ImageAnnotationBase {
  virtual void render() override;
  virtual void heatmap(QPainter& painter, const QColor& color) override;
};
DECLARE_TYPE(PointImageAnnotation, ImageAnnotationBase);