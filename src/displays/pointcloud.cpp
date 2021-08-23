// TAMSVIZ
// (c) 2020-2021 Philipp Ruppel

#include "pointcloud.h"

#include <sensor_msgs/point_cloud_conversion.h>

#include "../core/document.h"
#include "../core/mparser.h"
#include "../core/mquery.h"

// shouldn't have used eigen
void resizeEigenVectorVector3f(std::vector<Eigen::Vector3f> &vector,
                               size_t size) {
  struct Vec3 {
    float x, y, z;
  };
  ((std::vector<Vec3> *)(&vector))->resize(size);
}

PointCloudDisplayBase::PointCloudDisplayBase() {
  _material_renderer->block().flags |= 2; // unlit
  _material_renderer->block().flags |= 4; // sbgr colors
  _material->color().r() = 1;
  _material->color().g() = 1;
  _material->color().b() = 1;
}

template <class T>
void fetchPointPositionData(size_t point_count, size_t point_step,
                            size_t offset_x, size_t offset_y, size_t offset_z,
                            const std::vector<uint8_t> &buffer,
                            std::vector<Eigen::Vector3f> &positions) {
  {
    PROFILER("allocate point positions");
    resizeEigenVectorVector3f(positions, point_count);
  }

  if (point_count == 0) {
    return;
  }

  if (sizeof(T) == 4 && (point_step % 4 == 0) && (offset_x % 4 == 0) &&
      (offset_y == offset_x + sizeof(T)) &&
      (offset_z == offset_y + sizeof(T))) {
    PROFILER("fetch point positions fast");
    auto *p_buf = reinterpret_cast<const uint32_t *>(buffer.data());
    auto *p_pos = reinterpret_cast<uint32_t *>(&positions[0].x());
    auto *p_pos_end = p_pos + point_count * 3;
    p_buf += offset_x / 4;
    point_step /= 4;
    while (p_pos < p_pos_end) {
      p_pos[0] = p_buf[0];
      p_pos[1] = p_buf[1];
      p_pos[2] = p_buf[2];
      p_buf += point_step;
      p_pos += 3;
    }
    return;
  }

  {
    PROFILER("fetch point positions");
    auto *p_buf = buffer.data();
    auto *p_pos = &positions[0].x();
    for (size_t i = 0; i < point_count; i++) {
      p_pos[0] = *reinterpret_cast<const T *>(p_buf + offset_x);
      p_pos[1] = *reinterpret_cast<const T *>(p_buf + offset_y);
      p_pos[2] = *reinterpret_cast<const T *>(p_buf + offset_z);
      p_buf += point_step;
      p_pos += 3;
    }
  }
}

void PointCloudDisplayBase::setPointCloud(
    const std::shared_ptr<const Message> &message) {

  PROFILER("set point cloud");

  if (message) {

    {
      PROFILER("extract frame id");
      static MessageQuery query("header.frame_id");
      MessageParser parser(message);
      node()->frame(query(parser).toString());
    }

    bool use_point_colors = pointColors();
    _mesh_renderer = node()->create<MeshRenderer>(
        std::make_shared<Mesh>([message, use_point_colors](MeshData &mesh) {

          std::shared_ptr<const sensor_msgs::PointCloud2> cloud;
          {
            PROFILER("instantiate point cloud");
            cloud = message->instantiate<sensor_msgs::PointCloud2>();
            if (!cloud) {
              auto cloud1 = message->instantiate<sensor_msgs::PointCloud>();
              if (cloud1) {
                auto cloud2 = std::make_shared<sensor_msgs::PointCloud2>();
                bool ok = sensor_msgs::convertPointCloudToPointCloud2(*cloud1,
                                                                      *cloud2);
                if (ok) {
                  cloud = cloud2;
                } else {
                  LOG_ERROR_THROTTLE(1, "failed to parse PointCloud2 message");
                }
              }
            }
            if (!cloud) {
              return;
            }
          }

          std::vector<sensor_msgs::PointField> fields;
          for (auto &field : cloud->fields) {
            static std::map<uint8_t, size_t> element_sizes = {
                {sensor_msgs::PointField::INT8, 1},
                {sensor_msgs::PointField::UINT8, 1},
                {sensor_msgs::PointField::INT16, 2},
                {sensor_msgs::PointField::UINT16, 2},
                {sensor_msgs::PointField::INT32, 4},
                {sensor_msgs::PointField::UINT32, 4},
                {sensor_msgs::PointField::FLOAT32, 4},
                {sensor_msgs::PointField::FLOAT64, 8},
            };
            auto it = element_sizes.find(field.datatype);
            if (it == element_sizes.end()) {
              LOG_WARN_THROTTLE(1, "unsupported pointcloud data type "
                                       << (int)field.datatype);
              continue;
            }
            size_t element_size = it->second;
            if (size_t(field.offset) + element_size > cloud->point_step) {
              LOG_WARN_THROTTLE(1, "invalid pointcloud field offset");
              continue;
            }
            fields.push_back(field);
          }

          auto findField = [&](const char *name) -> sensor_msgs::PointField * {
            for (auto &field : fields) {
              if (field.name == name) {
                return &field;
              }
            }
            return nullptr;
          };

          auto x_field = findField("x");
          auto y_field = findField("y");
          auto z_field = findField("z");
          if (!x_field || !y_field || !z_field) {
            LOG_ERROR_THROTTLE(1, "no point position attribute");
            return;
          }

          if (x_field->datatype != y_field->datatype ||
              x_field->datatype != z_field->datatype) {
            LOG_ERROR_THROTTLE(1, "position datatypes incompatible");
            return;
          }

          size_t point_count = cloud->width * cloud->height;
          point_count =
              std::min(point_count, cloud->data.size() / cloud->point_step);

          switch (x_field->datatype) {
          case sensor_msgs::PointField::FLOAT32:
            fetchPointPositionData<float>(
                point_count, cloud->point_step, x_field->offset,
                y_field->offset, z_field->offset, cloud->data, mesh.positions);
            break;
          case sensor_msgs::PointField::FLOAT64:
            fetchPointPositionData<double>(
                point_count, cloud->point_step, x_field->offset,
                y_field->offset, z_field->offset, cloud->data, mesh.positions);
            break;
          default:
            LOG_ERROR_THROTTLE(1, "unsupported point position datatype");
            return;
          }

          if (use_point_colors) {
            auto color_field = findField("rgb");
            if (color_field &&
                (color_field->datatype == sensor_msgs::PointField::FLOAT32 ||
                 color_field->datatype == sensor_msgs::PointField::INT32 ||
                 color_field->datatype == sensor_msgs::PointField::UINT32)) {
              {
                PROFILER("allocate point colors");
                mesh.colors8.resize(point_count);
              }
              {
                PROFILER("convert point colors");
                size_t point_step = cloud->point_step;
                size_t field_offset = color_field->offset;
                auto *buffer_pointer = cloud->data.data() + field_offset;
                auto *color_begin = mesh.colors8.data();
                auto *color_end = (color_begin + point_count);
                for (auto *color_pointer = color_begin;
                     color_pointer < color_end; color_pointer++) {
                  uint32_t color =
                      *reinterpret_cast<const uint32_t *>(buffer_pointer);
                  *color_pointer = color;
                  buffer_pointer += point_step;
                }
              }
            }
          }

        }),
        _material_renderer);

    RenderOptions options;
    options.primitive_type = GL_POINTS;
    options.point_size = pointSize();
    _mesh_renderer->options(options);

  } else {

    _mesh_renderer.reset();
  }
}

void PointCloudDisplay::renderSync(const RenderSyncContext &context) {
  {
    PROFILER("pointcloud sync");
    std::shared_ptr<const Message> message;
    {
      PROFILER("fetch message");
      if (auto sub = topic().subscriber()) {
        if (auto topic = sub->topic()) {
          message = topic->message();
        }
      }
    }
    if (_watcher.changed(message, pointColors())) {
      setPointCloud(message);
    }
  }
  {
    PROFILER("pointcloud base");
    PointCloudDisplayBase::renderSync(context);
  }
}

void PointCloud2Display::renderSync(const RenderSyncContext &context) {
  {
    PROFILER("pointcloud sync");
    std::shared_ptr<const Message> message;
    {
      PROFILER("fetch message");
      if (auto sub = topic().subscriber()) {
        if (auto topic = sub->topic()) {
          message = topic->message();
        }
      }
    }
    if (_watcher.changed(message, pointColors())) {
      setPointCloud(message);
    }
  }
  {
    PROFILER("pointcloud base");
    PointCloudDisplayBase::renderSync(context);
  }
}
