// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "plotwindow.h"

#include "../core/bagplayer.h"
#include "../core/log.h"
#include "../core/topic.h"
#include "../core/workspace.h"

#include <QGraphicsScene>
#include <QGraphicsView>

#include <condition_variable>
#include <mutex>
#include <thread>

#include <std_msgs/Float64.h>

template <class Message> class TimeSeriesSubscriber;

template <class Message> class TimeSeries {
  std::deque<std::shared_ptr<const Message>> _messages;

public:
  TimeSeries() {}
  template <class MessageIterator>
  TimeSeries(const MessageIterator &messages_begin,
             const MessageIterator &messages_end)
      : _messages(messages_begin, messages_end) {}
  template <class MessageIterator>
  void assign(const MessageIterator &messages_begin,
              const MessageIterator &messages_end) {
    _messages.assign(messages_begin, messages_end);
  }
  auto begin() const -> decltype(_messages.cbegin()) {
    return _messages.cbegin();
  }
  auto end() const -> decltype(_messages.cend()) { return _messages.cend(); }
  size_t size() const { return _messages.size(); }
  bool empty() const { return _messages.empty(); }
  const std::shared_ptr<const Message> &front() const {
    return _messages.front();
  }
  const std::shared_ptr<const Message> &back() const {
    return _messages.back();
  }
  friend class TimeSeriesSubscriber<Message>;
};

template <class Message> class TimeSeriesSubscriber {
  struct Impl {
    std::function<void(const TimeSeries<Message> &)> callback_function;
    std::shared_ptr<void> callback_instance;
    TimeSeries<Message> time_series;
    std::shared_ptr<Message> received_message;
    bool stop_flag = false;
    std::mutex mutex;
    std::condition_variable condition;
    std::string topic;
    double duration = 0.0;
    void startThread(const std::shared_ptr<Impl> &shared) {
      std::thread([shared, this]() {
        shared;
        while (true) {
          LOG_DEBUG("time series thread loop");
          std::shared_ptr<Message> msg;
          double duration = 0.0;
          {
            std::unique_lock<std::mutex> lock(mutex);
            while (true) {
              LOG_DEBUG("time series thread wait loop");
              if (stop_flag) {
                LOG_DEBUG("time series thread exit");
                return;
              }
              if (received_message) {
                LOG_DEBUG("time series thread recv");
                msg = received_message;
                received_message = nullptr;
                duration = this->duration;
                break;
              }
              condition.wait(lock);
            }
          }
          std::shared_ptr<BagPlayer> player;
          double bag_time = 0;
          {
            LockScope ws;
            if (ws->player) {
              player = ws->player;
              bag_time = player->time();
            }
          }
          if (player) {
            auto samples = player->readMessageSamples(
                topic, bag_time - duration * 0.5, bag_time + duration * 0.5,
                duration * 0.01);
            // LOG_DEBUG(samples.size() << " samples");
            time_series.assign(samples.begin(), samples.end());
            callback_function(time_series);
          } else {
            time_series._messages.emplace_back(msg);
            while (time_series.size() >= 2 &&
                   (time_series.back()->time() -
                    (*(time_series.begin() + 1))->time())
                           .toSec() > duration) {
              time_series._messages.pop_front();
            }
            callback_function(time_series);
          }
        }
      })
          .detach();
    }
    void stopThread() {
      std::unique_lock<std::mutex> lock(mutex);
      stop_flag = true;
      condition.notify_all();
    }
    void handleMessage(const std::shared_ptr<Message> &message) {
      std::unique_lock<std::mutex> lock(mutex);
      received_message = message;
      condition.notify_one();
    }
  };
  std::shared_ptr<Impl> _impl = std::make_shared<Impl>();
  std::shared_ptr<Subscriber<Message>> _subscriber;

public:
  ~TimeSeriesSubscriber() {
    _impl->stopThread();
    _impl = nullptr;
  }
  TimeSeriesSubscriber(const std::string &topic,
                       const std::shared_ptr<void> &callback_instance,
                       const std::function<void(const TimeSeries<Message> &)>
                           callback_function) {
    _impl->topic = topic;
    _impl->callback_instance = callback_instance;
    _impl->callback_function = callback_function;
    _impl->startThread(_impl);
    _subscriber = std::make_shared<Subscriber<Message>>(
        topic, _impl,
        std::bind(&Impl::handleMessage, _impl.get(), std::placeholders::_1));
  }
  TimeSeriesSubscriber(const TimeSeriesSubscriber &) = delete;
  TimeSeriesSubscriber &operator=(const TimeSeriesSubscriber &) = delete;
  const std::string &topic() const { return _impl->topic; }
  const double duration() const { return _impl->duration; }
  void duration(double duration) {
    std::unique_lock<std::mutex> lock(_impl->mutex);
    _impl->duration = duration;
  }
};

PlotWindow::PlotWindow() {

  typedef TimeSeriesSubscriber<MessageType> SubscriberType;

  class GraphicsView : public QWidget {
    std::shared_ptr<TimeSeriesSubscriber<Message>> _subscriber;
    struct Data {
      QPointer<GraphicsView> parent;
      std::mutex mutex;
      QVector<QPointF> points;
      double duration = 0.0;
      void update(const TimeSeries<Message> &time_series) {
        auto ref_time = time_series.back()->time();
        bool from_bag = false;
        {
          LockScope ws;
          if (ws->player) {
            ref_time =
                ws->player->startTime() + ros::Duration(ws->player->time());
            from_bag = true;
          }
        }
        QPointer<GraphicsView> parent;
        {
          std::unique_lock<std::mutex> lock(mutex);
          if (from_bag) {
            ref_time += ros::Duration(duration * 0.5);
          }
          points.clear();
          for (auto &msg : time_series) {
            if (auto m = msg->instantiate<std_msgs::Float64>()) {
              double x = -(ref_time - msg->time()).toSec();
              double y = m->data;
              points.push_back(QPointF(x, y));
            }
          }
          parent = this->parent;
        }
        startOnMainThreadAsync([parent]() {
          if (auto *ptr = parent.data()) {
            ptr->update();
          }
        });
      }
    };
    std::shared_ptr<Data> _data = std::make_shared<Data>();
    PlotWindow *_parent = nullptr;
    void sync() {
      // LOG_DEBUG("plot sync");
      LockScope ws;
      {
        std::unique_lock<std::mutex> lock(_data->mutex);
        _data->parent = this;
        _data->duration = _parent->duration();
      }
      if (_parent->topic().empty() && _subscriber) {
        _subscriber = nullptr;
      }
      if (!_parent->topic().empty() &&
          (_subscriber == nullptr ||
           _subscriber->topic() != _parent->topic())) {
        _subscriber = std::make_shared<TimeSeriesSubscriber<Message>>(
            _parent->topic(), _data,
            std::bind(&Data::update, _data.get(), std::placeholders::_1));
      }
      if (_subscriber) {
        _subscriber->duration(_parent->duration());
      }
      update();
    }

    /*class GraphicsScene : public QGraphicsScene {
      GraphicsView *_parent = nullptr;

    public:
      GraphicsScene(GraphicsView *parent)
          : QGraphicsScene(parent), _parent(parent) {
        setSceneRect(0, 0, 100, 100);
      }
    };
    GraphicsScene *_scene = nullptr;*/

  private:
  public:
    GraphicsView(PlotWindow *parent) : _parent(parent) {
      LockScope()->modified.connect(this, [this]() {
        // LOG_DEBUG("modified");
        sync();
      });
      sync();
    }
    virtual void wheelEvent(QWheelEvent *wheel) override {}
    virtual void mouseMoveEvent(QMouseEvent *event) override {}
    virtual void mousePressEvent(QMouseEvent *event) override {
      LockScope ws;
      ws->selection() = _parent->shared_from_this();
      ws->modified();
    }
    virtual void mouseReleaseEvent(QMouseEvent *event) override {}
    virtual void paintEvent(QPaintEvent *event) override {
      // LOG_DEBUG("draw plot");
      // QPainter painter(viewport());
      QPainter painter(this);
      painter.fillRect(0, 0, width(), height(), QColor(Qt::white));
      double t = 0.0;
      double duration = 0.0;
      bool from_bag = false;
      {
        LockScope ws;
        if (ws->player) {
          t = ws->player->time();
          from_bag = true;
        }
        duration = _parent->duration();
      }
      if (from_bag) {
        painter.setPen(QPen(QBrush(QColor(150, 150, 150)), 0));
        painter.drawLine(width() * 0.5, 0, width() * 0.5, height());
      }
      QVector<QPointF> points;
      {
        std::unique_lock<std::mutex> lock(_data->mutex);
        points = _data->points;
      }
      if (!points.isEmpty()) {
        double lo = points[0].y();
        double hi = points[0].y();
        for (auto &p : points) {
          lo = std::min(lo, p.y());
          hi = std::max(hi, p.y());
        }
        for (auto &p : points) {
          // p.setX((p.x() - t + 0.5) * width());
          p.setX((1.0 + p.x() / std::max(1e-6, duration)) * width());
          p.setY((0.9 - (p.y() - lo) / (hi - lo + 1e-6) * 0.8) * height());
        }
      }
      QPolygonF polygon = QPolygonF(points);
      painter.save();
      // painter.resetTransform();
      painter.setRenderHints(QPainter::Antialiasing);
      painter.setPen(QPen(QBrush(Qt::black), 1.5, Qt::SolidLine, Qt::SquareCap,
                          Qt::RoundJoin));
      painter.setBrush(QBrush(Qt::transparent));
      painter.drawPolyline(polygon);
      painter.restore();
    }
  };

  auto *view = new GraphicsView(this);
  setContentWidget(view);
}
