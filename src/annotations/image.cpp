// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "image.h"

#include "../core/log.h"

#include <QPainterPath>
#include <QTransform>

struct PointImageAnnotation : ImageAnnotationBase {
  virtual void render() override {
    if (controlPoints().empty()) {
      shape = nullptr;
      complete = false;
    } else {
      shape = std::make_shared<QPainterPath>();
      shape->addPolygon(QPolygonF(QRectF(-10, -2, 20, 4))
                            .united(QPolygonF(QRectF(-2, -10, 4, 20))));
      shape->translate(controlPoints().front().x(),
                       controlPoints().front().y());
      complete = true;
    }
  }
};
DECLARE_TYPE(PointImageAnnotation, ImageAnnotationBase);

struct RectangleImageAnnotation : ImageAnnotationBase {
  virtual void render() override {
    if (controlPoints().size() < 2) {
      shape = nullptr;
      complete = false;
    } else {
      shape = std::make_shared<QPainterPath>();
      double left =
          std::min(controlPoints().at(0).x(), controlPoints().at(1).x());
      double right =
          std::max(controlPoints().at(0).x(), controlPoints().at(1).x());
      double top =
          std::min(controlPoints().at(0).y(), controlPoints().at(1).y());
      double bottom =
          std::max(controlPoints().at(0).y(), controlPoints().at(1).y());
      shape->addRect(left, top, right - left, bottom - top);
      complete = true;
    }
  }
};
DECLARE_TYPE(RectangleImageAnnotation, ImageAnnotationBase);

struct PolygonImageAnnotation : ImageAnnotationBase {
  virtual void render() override {
    shape = std::make_shared<QPainterPath>();
    QVector<QPointF> points;
    if (!controlPoints().empty()) {
      for (auto &p : controlPoints()) {
        points.push_back(QPointF(p.x(), p.y()));
      }
      {
        auto p = controlPoints().front();
        points.push_back(QPointF(p.x(), p.y()));
      }
    }
    shape->addPolygon(points);
    complete = ((controlPoints().size() > 3) &&
                (controlPoints().front() == controlPoints().back()));
  }
};
DECLARE_TYPE(PolygonImageAnnotation, ImageAnnotationBase);

struct TriangleImageAnnotation : ImageAnnotationBase {
  virtual void render() override {
    shape = std::make_shared<QPainterPath>();
    QVector<QPointF> points;
    if (!controlPoints().empty()) {
      for (auto &p : controlPoints()) {
        points.push_back(QPointF(p.x(), p.y()));
      }
      {
        auto p = controlPoints().front();
        points.push_back(QPointF(p.x(), p.y()));
      }
    }
    shape->addPolygon(points);
    complete = (controlPoints().size() >= 3);
  }
};
DECLARE_TYPE(TriangleImageAnnotation, ImageAnnotationBase);

struct QuadImageAnnotation : ImageAnnotationBase {
  virtual void render() override {
    shape = std::make_shared<QPainterPath>();
    QVector<QPointF> points;
    if (!controlPoints().empty()) {
      for (auto &p : controlPoints()) {
        points.push_back(QPointF(p.x(), p.y()));
      }
      {
        auto p = controlPoints().front();
        points.push_back(QPointF(p.x(), p.y()));
      }
    }
    shape->addPolygon(points);
    complete = (controlPoints().size() >= 4);
  }
};
DECLARE_TYPE(QuadImageAnnotation, ImageAnnotationBase);

struct CircleCenteredImageAnnotation : ImageAnnotationBase {
  virtual void render() override {
    if (controlPoints().size() < 2) {
      shape = nullptr;
      complete = false;
    } else {
      shape = std::make_shared<QPainterPath>();
      double cx = controlPoints().at(0).x();
      double cy = controlPoints().at(0).y();
      double r = (controlPoints().at(0) - controlPoints().at(1)).norm();
      shape->addEllipse(cx - r, cy - r, 2 * r, 2 * r);
      complete = true;
    }
  }
};
DECLARE_TYPE(CircleCenteredImageAnnotation, ImageAnnotationBase);

struct CircleEdgeImageAnnotation : ImageAnnotationBase {
  virtual void render() override {
    if (controlPoints().size() < 2) {
      shape = nullptr;
      complete = false;
    } else {
      shape = std::make_shared<QPainterPath>();
      double cx = (controlPoints().at(0).x() + controlPoints().at(1).x()) * 0.5;
      double cy = (controlPoints().at(0).y() + controlPoints().at(1).y()) * 0.5;
      double r = (controlPoints().at(0) - controlPoints().at(1)).norm() * 0.5;
      shape->addEllipse(cx - r, cy - r, 2 * r, 2 * r);
      complete = true;
    }
  }
};
DECLARE_TYPE(CircleEdgeImageAnnotation, ImageAnnotationBase);

struct EllipseRectImageAnnotation : ImageAnnotationBase {
  virtual void render() override {
    if (controlPoints().size() < 2) {
      shape = nullptr;
      complete = false;
    } else {
      shape = std::make_shared<QPainterPath>();
      double left =
          std::min(controlPoints().at(0).x(), controlPoints().at(1).x());
      double right =
          std::max(controlPoints().at(0).x(), controlPoints().at(1).x());
      double top =
          std::min(controlPoints().at(0).y(), controlPoints().at(1).y());
      double bottom =
          std::max(controlPoints().at(0).y(), controlPoints().at(1).y());
      shape->addEllipse(left, top, right - left, bottom - top);
      complete = true;
    }
  }
};
DECLARE_TYPE(EllipseRectImageAnnotation, ImageAnnotationBase);

struct EllipseCenteredImageAnnotation : ImageAnnotationBase {
  virtual void constrain() override {
    if (controlPoints().size() == 3) {
      double r = (controlPoints().at(0) - controlPoints().at(2)).norm();
      auto d = (controlPoints().at(0) - controlPoints().at(1)).normalized();
      controlPoints().at(2) +=
          d * d.dot(controlPoints().at(0) - controlPoints().at(2));
      controlPoints().at(2) =
          controlPoints().at(0) +
          (controlPoints().at(2) - controlPoints().at(0)).normalized() * r;
    }
  }
  virtual void render() override {
    if (controlPoints().size() < 2) {
      shape = nullptr;
      complete = false;
    } else if (controlPoints().size() == 2) {
      shape = std::make_shared<QPainterPath>();
      double cx = controlPoints().at(0).x();
      double cy = controlPoints().at(0).y();
      double r = (controlPoints().at(0) - controlPoints().at(1)).norm();
      shape->addEllipse(cx - r, cy - r, 2 * r, 2 * r);
      complete = false;
    } else {
      shape = std::make_shared<QPainterPath>();
      double width = (controlPoints().at(0) - controlPoints().at(1)).norm() * 2;
      auto d = (controlPoints().at(0) - controlPoints().at(1)).normalized();
      double height =
          std::abs(Eigen::Vector2d(-d.y(), d.x())
                       .dot(controlPoints().at(0) - controlPoints().at(2))) *
          2.0;
      shape->addEllipse(-width * 0.5, -height * 0.5, width, height);
      QTransform transform;
      transform.translate(controlPoints().at(0).x(), controlPoints().at(0).y());
      transform.rotate(
          std::atan2(controlPoints().at(0).y() - controlPoints().at(1).y(),
                     controlPoints().at(0).x() - controlPoints().at(1).x()) *
          (180.0 / M_PI));
      *shape = transform.map(*shape);
      complete = true;
    }
  }
};
DECLARE_TYPE(EllipseCenteredImageAnnotation, ImageAnnotationBase);
