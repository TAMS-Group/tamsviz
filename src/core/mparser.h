// TAMSVIZ
// (c) 2020 Philipp Ruppel

#pragma once

#include "message.h"
#include "profiler.h"

#include <typeindex>
#include <unordered_map>
#include <vector>

class MessageParser {

  struct Node {
    std::string hash;
    virtual void index(const MessageParser &parser,
                       size_t &data_offset) const = 0;
    virtual bool hasFixedSize() const = 0;
    virtual size_t fixedSize() const {
      throw std::runtime_error("Node does not have a fixed size");
    }
    virtual MessageParser elementAt(const MessageParser &parser,
                                    size_t index) const {
      return MessageParser();
    }
    virtual ssize_t fieldIndex(const MessageParser &parser,
                               const std::string &name) const {
      return -1;
    }
    virtual void init(const MessageParser &parser) {}
    virtual size_t elementCount(const MessageParser &parser) const { return 0; }
    virtual std::string toString(const MessageParser &parser) const {
      return "";
    }
    virtual double toDouble(const MessageParser &parser) const { return 0; }
    virtual int64_t toInteger(const MessageParser &parser) const { return 0; }
  };

  struct PrimitiveNode : MessageParser::Node {
    std::string name;
  };

  struct TimeNode : PrimitiveNode {
    std::string name;
    virtual bool hasFixedSize() const override { return true; }
    virtual size_t fixedSize() const override { return 8; }
    virtual void index(const MessageParser &parser,
                       size_t &start) const override {
      start += sizeof(fixedSize());
    }
    ros::Time value(const MessageParser &parser) const {
      uint32_t sec = parser.data<uint32_t>(parser._data_start + 0);
      uint32_t nsec = parser.data<uint32_t>(parser._data_start + 4);
      return ros::Time(sec, nsec);
    }
    virtual double toDouble(const MessageParser &parser) const override {
      return value(parser).toSec();
    }
    virtual int64_t toInteger(const MessageParser &parser) const override {
      return value(parser).toNSec();
    }
    /*virtual std::string toString(const MessageParser &parser) const override {
      return std::to_string(toInteger(parser));
  }*/
  };

  struct DurationNode : PrimitiveNode {
    std::string name;
    virtual bool hasFixedSize() const override { return true; }
    virtual size_t fixedSize() const override { return 8; }
    virtual void index(const MessageParser &parser,
                       size_t &start) const override {
      start += sizeof(fixedSize());
    }
    ros::Duration value(const MessageParser &parser) const {
      uint32_t sec = parser.data<uint32_t>(parser._data_start + 0);
      uint32_t nsec = parser.data<uint32_t>(parser._data_start + 4);
      return ros::Duration(sec, nsec);
    }
    virtual double toDouble(const MessageParser &parser) const override {
      return value(parser).toSec();
    }
    virtual int64_t toInteger(const MessageParser &parser) const override {
      return value(parser).toNSec();
    }
    /*virtual std::string toString(const MessageParser &parser) const override {
      return std::to_string(toInteger(parser));
  }*/
  };

  struct StringNode : PrimitiveNode {
    std::string name;
    virtual bool hasFixedSize() const override { return false; }
    virtual void index(const MessageParser &parser,
                       size_t &data_offset) const override {
      auto len = parser.data<uint32_t>(data_offset);
      data_offset += 4 + len;
    }
    virtual std::string toString(const MessageParser &parser) const override {
      size_t len = parser.data<uint32_t>(parser._data_start);
      return std::string((const char *)parser.data(parser._data_start + 4, len),
                         len);
    }
  };

  struct ArrayNode : Node {
    bool has_fixed_element_count = false;
    size_t fixed_element_count = 0;
    std::shared_ptr<Node> element_node;
    virtual bool hasFixedSize() const override {
      return has_fixed_element_count && element_node->hasFixedSize();
    }
    virtual size_t fixedSize() const override {
      return fixed_element_count * element_node->fixedSize();
    }
    virtual void index(const MessageParser &parser,
                       size_t &data_offset) const override {
      size_t element_count = fixed_element_count;
      if (!has_fixed_element_count) {
        element_count = parser.data<uint32_t>(data_offset);
        data_offset += sizeof(uint32_t);
      }
      if (element_node->hasFixedSize()) {
        data_offset += element_node->fixedSize() * element_count;
      } else {
        auto *index = parser._data->index.get();
        size_t index_start = index->alloc(element_count * 2);
        for (size_t i = 0; i < element_count; i++) {
          index->at(index_start + i * 2 + 0) = index->size();
          index->at(index_start + i * 2 + 1) = data_offset;
          element_node->index(parser, data_offset);
        }
      }
    }
    virtual void init(const MessageParser &parser) override {
      element_node->init(parser);
    }
    virtual size_t elementCount(const MessageParser &parser) const override {
      if (has_fixed_element_count) {
        return fixed_element_count;
      } else {
        return parser.data<uint32_t>(parser._data_start);
      }
    }
    virtual MessageParser elementAt(const MessageParser &parser,
                                    size_t index) const override {
      MessageParser ret;
      size_t element_count = fixed_element_count;
      size_t data_start = parser._data_start;
      if (!has_fixed_element_count) {
        element_count = parser.data<uint32_t>(parser._data_start);
        data_start += sizeof(uint32_t);
      }
      if (index < element_count) {
        ret._data = parser._data;
        ret._node = element_node;
        if (element_node->hasFixedSize()) {
          ret._index_start = parser._index_start;
          ret._data_start = data_start + element_node->fixedSize() * index;
        } else {
          ret._index_start =
              parser._data->index->at(parser._index_start + index * 2 + 0);
          ret._data_start =
              parser._data->index->at(parser._index_start + index * 2 + 1);
        }
      }
      return ret;
    }
  };

  struct ConstNode {
    std::string type_name;
    std::string name;
    std::string value_string;
  };

  struct MessageNode : Node {
    std::string name;
    std::vector<std::shared_ptr<Node>> field_nodes;
    std::vector<std::string> field_names;
    std::unordered_map<std::string, size_t> field_map;
    std::vector<std::shared_ptr<ConstNode>> const_nodes;
    std::unordered_map<std::string, size_t> const_map;
    bool fixed_size_initialized = false;
    bool has_fixed_size = false;
    size_t fixed_size = 0;
    std::vector<size_t> fixed_offsets;
    virtual void init(const MessageParser &parser) override {
      for (auto &field : field_nodes) {
        field->init(parser);
      }
      has_fixed_size = true;
      for (auto &field : field_nodes) {
        if (!field->hasFixedSize()) {
          has_fixed_size = false;
          break;
        }
      }
      fixed_size = 0;
      fixed_offsets.clear();
      if (has_fixed_size) {
        for (auto &field : field_nodes) {
          fixed_offsets.push_back(fixed_size);
          fixed_size += field->fixedSize();
        }
      }
      fixed_size_initialized = true;
    }
    virtual bool hasFixedSize() const override {
      if (!fixed_size_initialized) {
        throw std::runtime_error("Fixed size not initialized");
      }
      return has_fixed_size;
    }
    virtual size_t fixedSize() const override {
      if (!fixed_size_initialized) {
        throw std::runtime_error("Fixed size not initialized");
      }
      return fixed_size;
    }
    virtual void index(const MessageParser &parser,
                       size_t &data_offset) const override {
      if (!fixed_size_initialized) {
        throw std::runtime_error("Fixed size not initialized");
      }
      if (has_fixed_size) {
        data_offset += fixed_size;
      } else {
        auto *index = parser._data->index.get();
        size_t index_start = index->alloc(field_nodes.size() * 2);
        for (size_t i = 0; i < field_nodes.size(); i++) {
          index->at(index_start + i * 2 + 0) = index->size();
          index->at(index_start + i * 2 + 1) = data_offset;
          field_nodes[i]->index(parser, data_offset);
        }
      }
    }
    virtual size_t elementCount(const MessageParser &parser) const override {
      return field_nodes.size();
    }
    virtual ssize_t fieldIndex(const MessageParser &parser,
                               const std::string &name) const override {
      auto it = field_map.find(name);
      if (it != field_map.end()) {
        return it->second;
      } else {
        return -1;
      }
    }
    virtual MessageParser elementAt(const MessageParser &parser,
                                    size_t index) const override {
      if (!fixed_size_initialized) {
        throw std::runtime_error("Fixed size not initialized");
      }
      MessageParser ret;
      if (index < field_nodes.size()) {
        ret._data = parser._data;
        ret._node = field_nodes.at(index);
        if (has_fixed_size) {
          ret._index_start = parser._index_start;
          ret._data_start = parser._data_start + fixed_offsets.at(index);
        } else {
          ret._index_start =
              parser._data->index->at(parser._index_start + index * 2 + 0);
          ret._data_start =
              parser._data->index->at(parser._index_start + index * 2 + 1);
        }
      }
      return ret;
    }
  };

  class Index {
    std::vector<size_t> _data;

  public:
    inline size_t alloc(size_t size) {
      size_t ret = _data.size();
      _data.resize(_data.size() + size, 0);
      return ret;
    }
    inline void clear() { _data.clear(); }
    inline size_t at(size_t index) const { return _data.at(index); }
    inline size_t &at(size_t index) { return _data.at(index); }
    // inline size_t at(size_t index) const { return _data[index]; }
    // inline size_t &at(size_t index) { return _data[index]; }
    inline void push(size_t v) { _data.push_back(v); }
    inline size_t size() const { return _data.size(); }
  };

  struct Data {
    std::shared_ptr<const Message> message;
    std::shared_ptr<Index> index;
    std::unordered_map<std::string, std::shared_ptr<Node>> type_map;
  };

  const void *data(size_t start, size_t size) const {
    if (_data->message->size() < start + size) {
      throw std::runtime_error("Index out of range");
    }
    return _data->message->data() + start;
  }

  template <class T> const T &data(size_t start) const {
    return *(const T *)data(start, sizeof(T));
  }

  void parseDefinition(const std::shared_ptr<const MessageType> &type);

  template <class T>
  void registerType(const std::string &name, const std::shared_ptr<T> &t) {
    t->name = name;
    _data->type_map[name] = t;
  }

  template <class T, class... Args>
  void registerType(const std::string &name, const Args &... args) {
    registerType(name, std::make_shared<T>(args...));
  }

  template <class T> void registerPrimitiveType(const std::string &name) {
    struct NodeType : PrimitiveNode {
      virtual bool hasFixedSize() const override { return true; }
      virtual size_t fixedSize() const override { return sizeof(T); }
      virtual void index(const MessageParser &parser,
                         size_t &data_offset) const override {
        data_offset += sizeof(T);
      }
      T value(const MessageParser &parser) const {
        return *(const T *)parser.data(parser._data_start, sizeof(T));
      }
      virtual std::string toString(const MessageParser &parser) const override {
        return std::to_string(value(parser));
      }
      virtual double toDouble(const MessageParser &parser) const {
        return value(parser);
      }
      virtual int64_t toInteger(const MessageParser &parser) const {
        return value(parser);
      }
    };
    auto node = std::make_shared<NodeType>();
    registerType(name, node);
  }

  std::shared_ptr<Node> findType(const std::string &name,
                                 const std::string &ns = "");

  std::shared_ptr<Data> _data;
  std::shared_ptr<const Node> _node;
  size_t _data_start = 0;
  size_t _index_start = 0;

public:
  MessageParser() {}
  MessageParser(const std::shared_ptr<const Message> &message) {
    parse(message);
  }
  void parse(const std::shared_ptr<const Message> &message);
  ssize_t fieldIndex(const std::string &name) const {
    if (isNull()) {
      return -1;
    } else {
      return _node->fieldIndex(*this, name);
    }
  }
  MessageParser at(size_t index) const {
    if (isNull()) {
      return MessageParser();
    } else {
      return _node->elementAt(*this, index);
    }
  }
  MessageParser operator[](size_t index) const { return at(index); }
  MessageParser operator[](const std::string &name) const {
    return at(fieldIndex(name));
  }
  bool isNull() const { return _data == nullptr || _node == nullptr; }
  bool isPrimitive() const {
    return !isNull() &&
           dynamic_cast<const PrimitiveNode *>(_node.get()) != nullptr;
  }
  bool isMessage() const {
    return !isNull() &&
           dynamic_cast<const MessageNode *>(_node.get()) != nullptr;
  }
  bool isArray() const {
    return !isNull() && dynamic_cast<const ArrayNode *>(_node.get()) != nullptr;
  }
  bool isString() const {
    return !isNull() &&
           dynamic_cast<const StringNode *>(_node.get()) != nullptr;
  }
  bool isTime() const {
    return !isNull() && dynamic_cast<const TimeNode *>(_node.get()) != nullptr;
  }
  size_t size() const { return _node->elementCount(*this); }
  const std::string &fieldName(size_t i) const {
    return dynamic_cast<const MessageNode *>(_node.get())->field_names.at(i);
  }
  void print(std::ostream &stream) const;
  std::string print() const;
  std::string toString() const {
    if (isNull()) {
      return "";
    } else {
      return _node->toString(*this);
    }
  }
  double toDouble() const {
    if (isNull()) {
      return 0;
    } else {
      return _node->toDouble(*this);
    }
  }
  int64_t toInteger() const {
    if (isNull()) {
      return 0;
    } else {
      return _node->toInteger(*this);
    }
  }
  operator bool() const { return !isNull(); }
};
