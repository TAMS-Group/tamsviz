// TAMSVIZ
// (c) 2020-2023 Philipp Ruppel

#include "transformer.h"

#include "profiler.h"
#include "timeseries.h"
#include "topic.h"
#include "workspace.h"

#include <geometry_msgs/TransformStamped.h>
#include <tf/tfMessage.h>
#include <tf2_msgs/TFMessage.h>

#include <unordered_map>
#include <unordered_set>

#include <eigen_conversions/eigen_msg.h>

class TransformMessageListener : public TimeSeriesListener {
  mutable std::mutex _mutex;
  const std::function<void(
      const std::shared_ptr<const geometry_msgs::TransformStamped> &)>
      _callback;
  std::unordered_map<
      std::string,
      std::map<int64_t, std::shared_ptr<const geometry_msgs::TransformStamped>>>
      _data;
  std::unordered_set<std::string> _modified;
  void _push(const std::shared_ptr<geometry_msgs::TransformStamped> &msg,
             int64_t message_time, int64_t start, int64_t end) {
    auto &sequence = _data[msg->child_frame_id];
    _modified.insert(msg->child_frame_id);
    sequence.emplace(message_time, msg);
    while (!sequence.empty() && sequence.begin()->first < start) {
      sequence.erase(sequence.begin());
    }
    while (!sequence.empty() && sequence.rbegin()->first > end) {
      auto it = sequence.end();
      --it;
      sequence.erase(it);
    }
  }

 public:
  TransformMessageListener(
      const std::function<
          void(const std::shared_ptr<const geometry_msgs::TransformStamped> &)>
          &callback)
      : _callback(callback) {}
  TransformMessageListener(const TransformMessageListener &) = delete;
  TransformMessageListener &operator=(const TransformMessageListener &) =
      delete;
  virtual void push(const std::shared_ptr<const Message> &message,
                    int64_t start, int64_t end) override {
    PROFILER("TransformMessageListener");
    std::unique_lock<std::mutex> lock(_mutex);
    if (auto msg = message->instantiate<tf2_msgs::TFMessage>()) {
      for (auto &tf : msg->transforms) {
        _push(std::make_shared<geometry_msgs::TransformStamped>(tf),
              message->time().toNSec(), start, end);
      }
    }
    if (auto msg = message->instantiate<tf::tfMessage>()) {
      for (auto &tf : msg->transforms) {
        _push(std::make_shared<geometry_msgs::TransformStamped>(tf),
              message->time().toNSec(), start, end);
      }
    }
  }
  virtual void commit() override {
    std::unique_lock<std::mutex> lock(_mutex);
    for (auto &m : _modified) {
      auto &sequence = _data[m];
      if (!sequence.empty()) {
        auto it = sequence.end();
        --it;
        _callback(it->second);
      }
    }
    _modified.clear();
  }
};

typedef Eigen::Transform<double, 3, Eigen::Isometry, Eigen::DontAlign>
    Isometry3du;

static void normalizeFrameName(std::string &name) {
  if (!name.empty() && name.front() == '/') {
    name.erase(0, 1);
  }
}

class FrameManager {
 public:
  std::mutex _mutex;
  std::unordered_map<std::string, int64_t> _references;
  static std::shared_ptr<FrameManager> instance() {
    static auto instance = std::make_shared<FrameManager>();
    return instance;
  }
  void count(std::string name, int64_t d) {
    normalizeFrameName(name);
    if (!name.empty()) {
      LOG_DEBUG("count frame " << name << " " << d);
      bool redraw = false;
      {
        std::lock_guard<std::mutex> lock(_mutex);
        int64_t &r = _references[name];
        // LOG_DEBUG("frame " << name << " refcount " << r << " d " << d);
        if (r == 0) {
          redraw = true;
        }
        r += d;
        // LOG_DEBUG("frame " << name << " refcount " << r << " d " << d);
        if (r < 0) {
          throw std::runtime_error("frame reference counter corrupted");
        }
      }
      if (redraw) {
        GlobalEvents::instance()->redraw();
      }
    }
  }
  bool check(std::string name) {
    normalizeFrameName(name);
    bool used = false;
    if (!name.empty()) {
      std::lock_guard<std::mutex> lock(_mutex);
      auto it = _references.find(name);
      if (it != _references.end()) {
        if (it->second > 0) {
          used = true;
        }
      }
    }
    return used;
  }
};

struct Transformer::Data {
  struct Connection {
    std::string parent;
    Eigen::Isometry3d transform;
  };
  std::mutex _mutex;
  std::vector<std::shared_ptr<void>> _subscribers;
  std::unordered_map<
      std::string, Connection, std::hash<std::string>,
      std::equal_to<std::string>,
      Eigen::aligned_allocator<std::pair<const std::string, Connection>>>
      _connections;
  std::unordered_map<
      std::string, Eigen::Isometry3d, std::hash<std::string>,
      std::equal_to<std::string>,
      Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d>>>
      _poses;
  // std::vector<size_t> _current;
  // std::vector<size_t> _next;
  // std::vector<uint8_t> _visited;
  // std::vector<ssize_t> _parents;
  std::set<std::string> _used;
  volatile bool _redraw_emitted = false;

  void clear() {
    std::unique_lock<std::mutex> lock(_mutex);
    _connections.clear();
    _poses.clear();
    // _current.clear();
    // _next.clear();
    // _visited.clear();
    // _parents.clear();
    _used.clear();
    _redraw_emitted = false;
  }

  void push(const geometry_msgs::TransformStamped &tf) {
    Eigen::Isometry3d transform;
    tf::transformMsgToEigen(tf.transform, transform);
    Eigen::Isometry3d transform_inverse = transform.inverse();
    if (transform.matrix().allFinite() &&
        transform_inverse.matrix().allFinite()) {
      bool _redraw = false;
      {
        std::unique_lock<std::mutex> lock(_mutex);
        auto parent = tf.header.frame_id;
        auto child = tf.child_frame_id;
        if (parent != child) {
          auto &conn = _connections[child];
          if (!conn.transform.isApprox(transform) || conn.parent != parent) {
            LOG_DEBUG("transform changed " << tf.header.frame_id << " "
                                           << tf.child_frame_id);
            if (_used.find(child) != _used.end()) {
              _redraw = true;
            }
            conn.parent = parent;
            conn.transform = transform;
          }
        }
      }
      if (_redraw) {
        if (!_redraw_emitted) {
          _redraw_emitted = true;
          GlobalEvents::instance()->redraw();
        }
      }
    }
  }

  void update(const std::string &root_name) {
    std::unique_lock<std::mutex> lock(_mutex);
    _redraw_emitted = false;
    _poses.clear();
    _poses[root_name] = Eigen::Isometry3d::Identity();
    bool any_changed = false;
    do {
      any_changed = false;
      for (auto &conn : _connections) {
        if (_poses.find(conn.first) == _poses.end() &&
            _poses.find(conn.second.parent) != _poses.end()) {
          _poses[conn.first] =
              _poses[conn.second.parent] * conn.second.transform;
          any_changed = true;
        }
      }
    } while (any_changed);
    {
      // LOG_DEBUG("mark frames");
      _used.clear();
      auto manager = FrameManager::instance();
      std::lock_guard<std::mutex> lock(manager->_mutex);
      for (auto &p : manager->_references) {
        if (p.second > 0) {
          std::string n = p.first;
          while (_used.find(n) == _used.end()) {
            _used.insert(n);
            auto conn = _connections.find(n);
            if (conn != _connections.end()) {
              n = conn->second.parent;
            } else {
              break;
            }
          }
        }
      }
    }
    // if (0) {
    //   for (size_t i = 0; i < _used.size(); i++) {
    //     if (_used[i]) {
    //       LOG_DEBUG(i << " " << _index_to_name[i] << " " << (int)_used[i]);
    //     }
    //   }
    // }
    // if (!any) {
    //   LOG_WARN_THROTTLE(1.0, "no connections from base frame: " <<
    //   root_name);
    // }
  }

  Optional<Eigen::Isometry3d> lookup(const std::string &name) {
    std::unique_lock<std::mutex> lock(_mutex);
    auto it = _poses.find(name);
    if (it != _poses.end()) {
      return it->second;
    } else {
      return Optional<Eigen::Isometry3d>();
    }
  }

  std::vector<std::string> list() {
    std::unordered_set<std::string> set;
    for (auto &conn : _connections) {
      set.insert(conn.first);
      set.insert(conn.second.parent);
    }
    std::vector<std::string> ret;
    for (auto n : set) {
      ret.push_back(n);
    }
    return ret;
  }

  void push(const std::shared_ptr<const Message> &message) {
    if (auto msg = message->instantiate<tf2_msgs::TFMessage>()) {
      for (auto &tf : msg->transforms) {
        push(tf);
      }
    }
    if (auto msg = message->instantiate<tf::tfMessage>()) {
      for (auto &tf : msg->transforms) {
        push(tf);
      }
    }
    if (auto msg = message->instantiate<geometry_msgs::TransformStamped>()) {
      push(*msg);
    }
  }
};

Transformer::Transformer(bool subscribe) {
  LOG_DEBUG("Transformer ctor");
  _data = std::make_shared<Data>();
  if (subscribe) {
    auto _data = this->_data;
    std::thread([_data]() {
      if (_data->_subscribers.empty()) {
        {
          auto topic = "/tf_static";
          auto *data = _data.get();
          auto subscriber = std::make_shared<Subscriber<Message>>(
              topic, _data,
              [data](const std::shared_ptr<const Message> &message) {
                PROFILER("Transformer");
                data->push(message);
              },
              false);
          subscriber->topic()->connected.connect(_data,
                                                 [data]() { data->clear(); });
          _data->_subscribers.push_back(subscriber);
        }
        {
          auto topic = "/tf";
          std::weak_ptr<Data> data = _data;
          auto subscriber = std::make_shared<TimeSeriesSubscriber>(topic);
          subscriber->duration(0.5);
          _data->_subscribers.push_back(subscriber);
          auto listener = std::make_shared<TransformMessageListener>(
              [data](
                  const std::shared_ptr<const geometry_msgs::TransformStamped>
                      &message) {
                if (auto d = data.lock()) {
                  PROFILER("Transformer");
                  d->push(*message);
                }
              });
          subscriber->addListener(listener);
          _data->_subscribers.push_back(listener);
        }
      }
    }).detach();
  }
}

Transformer::~Transformer() { LOG_DEBUG("Transformer dtor"); }

void Transformer::clear() { _data->clear(); }

void Transformer::update(const std::string &root) {
  _root_name = root;
  _data->update(root);
}

void Transformer::push(const std::shared_ptr<const Message> &message) {
  _data->push(message);
}

std::vector<std::string> Transformer::list() { return _data->list(); }

/*
Optional<Eigen::Isometry3d> Transformer::lookup(const Frame &frame) {
  return _data->lookup(frame.name());
}
*/

Frame::Frame() {}

Frame::Frame(const std::string &name) : _name(name) {}

Frame::Frame(const Frame &other) { _name = other._name; }

Frame &Frame::operator=(const Frame &other) {
  if (other._name != _name) {
    if (_active) {
      FrameManager::instance()->count(other._name, +1);
      FrameManager::instance()->count(_name, -1);
    }
    _name = other._name;
  }
  return *this;
}

void Frame::name(const std::string &name) {
  if (name != _name) {
    if (_active) {
      FrameManager::instance()->count(name, +1);
      FrameManager::instance()->count(_name, -1);
    }
    _name = name;
  }
}

Frame::~Frame() {
  if (_active) {
    FrameManager::instance()->count(_name, -1);
  }
}

Optional<Eigen::Isometry3d> Frame::pose(
    const std::shared_ptr<Transformer> &transformer) {
  if (!_active) {
    _active = true;
    FrameManager::instance()->count(_name, +1);
  }
  return transformer->_data->lookup(_name);
}

void DefaultPropertyAttributes<Frame>::initialize(
    PropertyAttributes *attributes) {
  attributes->list = [](const Property &property) {
    return LockScope()->document()->display()->transformer->list();
  };
}
