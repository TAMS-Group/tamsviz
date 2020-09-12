// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "plot.h"

#include "../core/profiler.h"

#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>
#include <tf2_msgs/TFMessage.h>

inline QColor toQColor(const Color3 &c) {
  return QColor::fromRgbF(c.r(), c.g(), c.b());
}

void PlotRenderer::PlotRendererData::update() {
  topics.resize(display->topics().size());
  for (size_t itopic = 0; itopic < topics.size(); itopic++) {
    auto &topic_display = display->topics()[itopic];
    auto &topic_renderer = topics[itopic];

    if (!topic_renderer.subscriber ||
        topic_renderer.subscriber->topic() != topic_display.topic.topic()) {
      topic_renderer.subscriber = topic_display.subscriber.lock();
      topic_renderer.queries.clear();
    }
    if (!topic_renderer.subscriber ||
        topic_renderer.subscriber->topic() != topic_display.topic.topic()) {
      topic_renderer.subscriber =
          std::make_shared<TimeSeriesSubscriber>(topic_display.topic.topic());
      topic_renderer.queries.clear();
    }
    topic_display.subscriber = topic_renderer.subscriber;
    topic_renderer.subscriber->duration(display->duration());

    topic_renderer.queries.resize(topic_display.queries.size());
    for (size_t iquery = 0; iquery < topic_renderer.queries.size(); iquery++) {
      auto &query_display = topic_display.queries[iquery];
      auto &query_renderer = topic_renderer.queries[iquery];
      query_renderer.color = query_display.color;
      if (query_renderer.query == nullptr ||
          query_renderer.query->query().str() != query_display.query.str()) {
        query_renderer.query =
            std::make_shared<TimeSeriesQuery>(query_display.query.query());
        topic_renderer.subscriber->addListener(query_renderer.query);
      }
      // query_display.query.subscriber(topic_renderer.subscriber->subscriber());
    }
  }
}

PlotRenderer::PlotRenderer(const std::shared_ptr<PlotDisplay> &display) {
  _data->display = display;
  auto *data = _data.get();
  LockScope()->modified.connect(_data, [data]() {
    LockScope ws;
    data->update();
  });
  data->update();
}

void PlotRenderer::renderSync() {
  PROFILER("PlotRenderer sync");
  _data->update();
  _duration = _data->display->duration();
  if (_data->display) {
    _style = _data->display->style();
  } else {
    _style = PlotStyle();
  }
  _topics_async = _data->topics;
  _bag_player = LockScope()->player;
}

inline double stepSize(double diff) {
  static std::vector<double> steps = {
      std::log10(1),
      std::log10(2),
      std::log10(5),
      std::log10(10),
  };
  double lg = std::log10(diff);
  double frac = lg - std::floor(lg);
  double step = steps[0];
  for (auto &s : steps) {
    if (std::abs(s - frac) < std::abs(step - frac)) {
      step = s;
    }
  }
  return std::pow(10.0, std::floor(lg) + step);
}

void PlotRenderer::renderAsync(QPainter *painter) {
  PROFILER("PlotRenderer async");
  bool init = false;
  int64_t tmin = 0, tmax = 0;
  double vmin = 0, vmax = 0;
  for (auto &topic : _topics_async) {
    for (auto &query : topic.queries) {
      query.points.clear();
      for (auto &pair : query.query->data()) {
        int64_t t = pair.first;
        double v = pair.second;
        query.points.emplace_back(t, v);
        if (!init) {
          init = true;
          tmin = tmax = t;
          vmin = vmax = v;
        } else {
          tmin = std::min(tmin, t);
          tmax = std::max(tmax, t);
          vmin = std::min(vmin, v);
          vmax = std::max(vmax, v);
        }
      }
    }
  }

  QRectF canvas = painter->viewport();
  painter->fillRect(canvas, toQColor(_style.backgroundColor));

  QRectF frame = canvas.marginsRemoved(QMarginsF(
      _style.margins.left +
          (_style.axes.y.label.empty()
               ? 0
               : (_style.axes.fontSize + _style.padding)) +
          _style.padding * 2 + _style.ticks.length + _style.ticks.y.width,
      _style.margins.top + _style.padding +
          ((!_style.title.enable || _style.title.text.empty() ||
            _style.title.fontSize == 0)
               ? 0
               : (_style.title.fontSize + _style.padding)),
      _style.margins.right + _style.padding,
      _style.margins.bottom +
          (_style.axes.x.label.empty()
               ? 0
               : (_style.axes.fontSize + _style.padding)) +
          _style.padding * 2 + _style.ticks.length + _style.ticks.fontSize));

  painter->setPen(QPen(toQColor(_style.foregroundColor), _style.frameWidth));
  painter->drawRect(frame);

  painter->setPen(
      QPen(toQColor(_style.foregroundColor), _style.axes.lineWidth));
  painter->drawLine(frame.bottomLeft(), frame.bottomRight());
  painter->drawLine(frame.bottomLeft(), frame.topLeft());

  if (!(!_style.title.enable || _style.title.text.empty() ||
        _style.title.fontSize == 0)) {
    QFont font;
    font.setPixelSize(_style.title.fontSize);
    painter->setFont(font);
    painter->drawText(QRectF(frame.left(), _style.margins.top, frame.width(),
                             _style.title.fontSize + _style.padding * 2),
                      Qt::AlignCenter, _style.title.text.c_str());
  }

  if (_style.axes.fontSize > 0) {
    QFont font;
    font.setPixelSize(_style.axes.fontSize);
    painter->setFont(font);

    painter->drawText(
        QRectF(frame.left(),
               canvas.bottom() - _style.axes.fontSize - _style.padding * 2,
               frame.width(), _style.axes.fontSize + _style.padding * 2),
        Qt::AlignCenter, _style.axes.x.label.c_str());

    painter->save();
    painter->translate(_style.margins.left, frame.bottom());
    painter->rotate(-90);
    painter->drawText(
        QRectF(0, 0, frame.height(), _style.axes.fontSize + _style.padding * 2),
        Qt::AlignCenter, _style.axes.y.label.c_str());
    painter->restore();
  }

  if (vmin == vmax) {
    vmin -= 1e-6;
    vmax += 1e-6;
  }

  if (_bag_player) {
    tmax = (_bag_player->startTime() + ros::Duration(_bag_player->time()))
               .toNSec();
  } else {
    tmax = ros::Time::now().toNSec();
  }

  tmin = tmax - std::max(int64_t(1000), int64_t(_duration * 1000000000));

  {
    double vd = (vmax - vmin) * (1.0 / frame.height());
    vmin -= vd * _style.spacing.bottom;
    vmax += vd * _style.spacing.top;
  }

  {
    double vd = (tmax - tmin) * (1.0 / frame.width());
    tmin -= vd * _style.spacing.left;
    tmax += vd * _style.spacing.right;
  }

  if (tmax != tmin && vmax != vmin) {

    QFont font;
    font.setPixelSize(_style.ticks.fontSize);
    painter->setFont(font);

    {
      int64_t step = std::round(
          stepSize((tmax - tmin) * (_style.ticks.x.stride * 1.0 /
                                    std::max(1.0, 1.0 * frame.width()))));
      for (int64_t x = (tmin + step - 1) / step * step; x <= tmax / step * step;
           x += step) {
        double fx = (x - tmin) * (1.0 / (tmax - tmin));
        double px = fx * frame.width() + frame.left();
        painter->setPen(
            QPen(toQColor(_style.foregroundColor), _style.ticks.width));
        painter->drawLine(px, frame.bottom(), px,
                          frame.bottom() + _style.ticks.length);
        if (_style.grid.enable) {
          painter->setPen(QPen(toQColor(_style.grid.color), _style.grid.width));
          painter->drawLine(px, frame.bottom(), px, frame.top());
        }
        painter->setPen(QPen(toQColor(_style.foregroundColor)));
        double w = step * 1.0 / (tmax - tmin) * frame.width();
        painter->drawText(
            QRectF(px - w * 0.5, frame.bottom() + _style.ticks.length, w,
                   _style.ticks.fontSize + _style.padding * 2),
            Qt::AlignCenter,
            QString::number((x % (1000000000l * 1000l)) * (1.0 / 1000000000),
                            'g', 2));
      }
    }

    {
      double step =
          stepSize((vmax - vmin) * (_style.ticks.y.stride * 1.0 /
                                    std::max(1.0, 1.0 * frame.height())));
      for (double y = vmin / step * step; y <= vmax / step * step; y += step) {
        double fy = 1.0 - (y - vmin) * (1.0 / (vmax - vmin));
        double py = fy * frame.height() + frame.top();
        painter->setPen(
            QPen(toQColor(_style.foregroundColor), _style.ticks.width));
        painter->drawLine(frame.left(), py, frame.left() - _style.ticks.length,
                          py);
        if (_style.grid.enable) {
          painter->setPen(QPen(toQColor(_style.grid.color), _style.grid.width));
          painter->drawLine(frame.left(), py, frame.right(), py);
        }
        painter->setPen(QPen(toQColor(_style.foregroundColor)));
        double h = step * 1.0 / (vmax - vmin) * frame.height();
        painter->drawText(
            QRectF(frame.left() - _style.ticks.length - _style.ticks.y.width -
                       _style.padding * 2,
                   py - h * 0.5, _style.ticks.y.width + _style.padding, h),
            Qt::AlignVCenter | Qt::AlignRight, QString::number(y, 'g', 3));
      }
    }

    double width = frame.width();
    double height = frame.height();
    for (auto &topic : _topics_async) {
      for (auto &query : topic.queries) {
        std::vector<QPointF> points;
        for (auto &p : query.points) {
          double fx = (p.first - tmin) * (1.0 / (tmax - tmin));
          double fy = 1.0 - (p.second - vmin) * (1.0 / (vmax - vmin));
          double px = fx * frame.width() + frame.left();
          double py = fy * frame.height() + frame.top();
          points.emplace_back(px, py);
        }
        painter->setPen(QPen(toQColor(query.color), _style.graphWidth));
        painter->drawPolyline(points.data(), points.size());
      }
    }
  }
}
