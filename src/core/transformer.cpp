// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "transformer.h"

#include "topic.h"

#include <geometry_msgs/TransformStamped.h>
#include <tf/tfMessage.h>
#include <tf2_msgs/TFMessage.h>

#include <unordered_map>
#include <unordered_set>

#include <eigen_conversions/eigen_msg.h>

typedef Eigen::Transform<double, 3, Eigen::Isometry, Eigen::DontAlign>
    Isometry3du;

struct Transformer::Data {

  std::mutex _mutex;
  std::vector<std::shared_ptr<void>> _subscribers;
  std::vector<std::string> _index_to_name;
  std::unordered_map<std::string, size_t> _name_to_index_map;
  std::unordered_map<size_t, std::unordered_map<size_t, Isometry3du>>
      _connections;
  std::vector<Optional<Eigen::Isometry3d>,
              Eigen::aligned_allocator<Optional<Eigen::Isometry3d>>>
      _poses;
  std::vector<size_t> _current;
  std::vector<size_t> _next;
  std::vector<uint8_t> _visited;

  void clear() {
    std::unique_lock<std::mutex> lock(_mutex);
    _index_to_name.clear();
    _name_to_index_map.clear();
    _connections.clear();
    _poses.clear();
    _current.clear();
    _next.clear();
    _visited.clear();
  }

  void normalizeName(std::string &name) {
    if (!name.empty() && name.front() == '/') {
      name.erase(0, 1);
    }
  }

  size_t nameToIndexCreate(std::string name) {
    normalizeName(name);
    {
      auto it = _name_to_index_map.find(name);
      if (it != _name_to_index_map.end()) {
        return it->second;
      }
    }
    _name_to_index_map[name] = _index_to_name.size();
    _index_to_name.push_back(name);
    {
      auto it = _name_to_index_map.find(name);
      if (it != _name_to_index_map.end()) {
        return it->second;
      }
    }
    throw std::runtime_error("");
  }

  ssize_t tryNameToIndex(std::string name) {
    normalizeName(name);
    {
      auto it = _name_to_index_map.find(name);
      if (it != _name_to_index_map.end()) {
        return it->second;
      }
    }
    return -1;
  }

  void push(const geometry_msgs::TransformStamped &tf) {
    Eigen::Isometry3d transform;
    tf::transformMsgToEigen(tf.transform, transform);
    Eigen::Isometry3d transform_inverse = transform.inverse();
    if (transform.matrix().allFinite() &&
        transform_inverse.matrix().allFinite()) {
      std::unique_lock<std::mutex> lock(_mutex);
      auto parent = nameToIndexCreate(tf.header.frame_id);
      auto child = nameToIndexCreate(tf.child_frame_id);
      _connections[parent][child] = transform;
      _connections[child][parent] = transform_inverse;
    }
  }

  void update(const std::string &root_name) {
    std::unique_lock<std::mutex> lock(_mutex);
    size_t root = nameToIndexCreate(root_name);
    _poses.clear();
    _poses.resize(_index_to_name.size(), Optional<Eigen::Isometry3d>());
    _poses[root] = Eigen::Isometry3d::Identity();
    _current.clear();
    _current.push_back(root);
    _visited.clear();
    _visited.resize(_index_to_name.size(), 0);
    bool any = false;
    while (!_current.empty()) {
      _next.clear();
      for (size_t curr : _current) {
        for (auto &conn : _connections[curr]) {
          if (!_visited[conn.first]) {
            any = true;
            _visited[conn.first] = true;
            _poses[conn.first] = *_poses[curr] * conn.second;
            _next.emplace_back(conn.first);
          }
        }
      }
      _current = _next;
    }
    if (!any) {
      LOG_WARN_THROTTLE(1.0, "no connections from base frame: " << root_name);
    }
  }

  Optional<Eigen::Isometry3d> lookup(const std::string &name) {
    std::unique_lock<std::mutex> lock(_mutex);
    ssize_t index = tryNameToIndex(name);
    if (index >= 0 && index < _poses.size()) {
      return _poses[index];
    } else {
      return Optional<Eigen::Isometry3d>();
    }
  }

  std::vector<std::string> list() {
    std::unordered_set<size_t> indices;
    {
      std::unique_lock<std::mutex> lock(_mutex);
      for (auto &a : _connections) {
        for (auto &b : a.second) {
          indices.insert(a.first);
          indices.insert(b.first);
        }
      }
    }
    std::vector<std::string> ret;
    for (auto i : indices) {
      ret.push_back(_index_to_name[i]);
    }
    return ret;
  }

  void push(const std::shared_ptr<Message> &message) {
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
  _data = std::make_shared<Data>();
  if (subscribe) {
    auto *data = _data.get();
    if (_data->_subscribers.empty()) {
      for (auto *topic : {"/tf", "/tf_static"}) {
        auto subscriber = std::make_shared<Subscriber<Message>>(
            topic, _data, [data](const std::shared_ptr<Message> &message) {
              data->push(message);
            });
        subscriber->topic()->connected.connect(_data,
                                               [data]() { data->clear(); });
        _data->_subscribers.push_back(subscriber);
      }
    }
  }
}

Optional<Eigen::Isometry3d> Transformer::lookup(const std::string &frame) {
  return _data->lookup(frame);
}

void Transformer::clear() { _data->clear(); }

void Transformer::update(const std::string &root) { _data->update(root); }

void Transformer::push(const std::shared_ptr<Message> &message) {
  _data->push(message);
}

std::vector<std::string> Transformer::list() { return _data->list(); }
