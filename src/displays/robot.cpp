// TAMSVIZ
// (c) 2020 Philipp Ruppel

#include "robot.h"

#include "../core/log.h"
#include "../core/transformer.h"
#include "../core/workspace.h"
#include "../render/mesh.h"
#include "shapes.h"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>

#include <geometric_shapes/shape_operations.h>

#include <assimp/Importer.hpp>
#include <assimp/config.h>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <eigen_conversions/eigen_msg.h>

static std::shared_ptr<Mesh>
createShapeMesh(const urdf::VisualSharedPtr &visual) {
  auto geometry = visual->geometry;
  switch (geometry->type) {
  case urdf::Geometry::SPHERE: {
    auto *sphere = (const urdf::Sphere *)geometry.get();
    return std::make_shared<Mesh>(Eigen::Scaling(float(sphere->radius)) *
                                  makeSphere(32, 16));
  }
  case urdf::Geometry::BOX: {
    auto *box = (const urdf::Box *)geometry.get();
    return std::make_shared<Mesh>(
        Eigen::Scaling(Eigen::Vector3f(box->dim.x * 0.5, box->dim.y * 0.5,
                                       box->dim.z * 0.5)) *
        makeBox());
  }
  case urdf::Geometry::CYLINDER: {
    auto *cylinder = (const urdf::Cylinder *)geometry.get();
    return std::make_shared<Mesh>(
        Eigen::Scaling(Eigen::Vector3f(cylinder->radius, cylinder->radius,
                                       cylinder->length * 0.5)) *
        makeCylinder(32));
  }
  }
  return nullptr;
}

struct RobotLink {
  std::shared_ptr<Material> material = []() {
    ObjectScope ws;
    return std::make_shared<Material>();
  }();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  const moveit::core::LinkModel *moveit_link = nullptr;
  size_t link_index = 0;
  size_t part_index = 0;
  Frame frame;
  std::shared_ptr<Mesh> mesh;
  RobotLink(const moveit::core::LinkModel *moveit_link,
            const Eigen::Isometry3d &pose, const std::shared_ptr<Mesh> &mesh)
      : moveit_link(moveit_link), link_index(moveit_link->getLinkIndex()),
        pose(pose), mesh(mesh), frame(moveit_link->getName()) {}
};

struct RobotModel {
  moveit::core::RobotModelConstPtr moveit_robot;
  std::vector<std::shared_ptr<RobotLink>> links;
  RobotModel(const std::string &name, RobotModelImportOptions import_options) {
    {
      robot_model_loader::RobotModelLoader moveit_loader(name, false);
      moveit_robot = moveit_loader.getModel();
      if (!moveit_robot) {
        LOG_ERROR("failed to load robot model " << name);
        return;
      }
    }
    auto urdf_model = moveit_robot->getURDF();
    for (auto &urdf_link_pair : urdf_model->links_) {
      auto urdf_link = urdf_link_pair.second;
      if (const moveit::core::LinkModel *moveit_link =
              moveit_robot->getLinkModel(urdf_link->name)) {
        for (auto &urdf_visual : urdf_link->visual_array) {
          if (urdf_visual->geometry) {
            size_t part_index = 0;
            auto rot = urdf_visual->origin.rotation;
            auto pos = urdf_visual->origin.position;
            Eigen::Isometry3d visual_pose =
                Eigen::Isometry3d(Eigen::Translation3d(pos.x, pos.y, pos.z)) *
                Eigen::Isometry3d(
                    Eigen::Quaterniond(rot.w, rot.x, rot.y, rot.z));
            if (auto mesh = createShapeMesh(urdf_visual)) {
              auto link = new RobotLink(moveit_link, visual_pose, mesh);
              if (auto mat = urdf_visual->material) {
                LockScope ws;
                LOG_DEBUG("urdf material color "
                          << urdf_link->name << " " << mat->color.r << " "
                          << mat->color.g << " " << mat->color.b << " "
                          << mat->color.a);
                link->material->color().r() = mat->color.r;
                link->material->color().g() = mat->color.g;
                link->material->color().b() = mat->color.b;
                link->material->color().a() = mat->color.a;
                link->material->texture() = mat->texture_filename;
              } else {
                LOG_DEBUG("no material for " << urdf_link->name);
              }
              links.emplace_back(link);
            } else if (urdf_visual->geometry->type == urdf::Geometry::MESH) {
              auto *urdf_mesh = (const urdf::Mesh *)urdf_visual->geometry.get();
              if (urdf_mesh->filename.empty()) {
                LOG_ERROR("empty mesh file name")
              } else {
                // LOG_INFO("loading mesh " << urdf_mesh->filename);
                std::string data;
                try {
                  loadResource(urdf_mesh->filename, data);
                } catch (std::exception &ex) {
                  LOG_ERROR("failed to read mesh file " << urdf_mesh->filename);
                }
                if (data.empty()) {
                  LOG_ERROR("failed to read mesh file " << urdf_mesh->filename);
                } else {
                  std::string assimp_hint;
                  if (auto *h =
                          std::strrchr(urdf_mesh->filename.c_str(), '.')) {
                    assimp_hint = h + 1;
                    for (auto &c : assimp_hint) {
                      c = std::tolower(c);
                    }
                    if (std::strstr(assimp_hint.c_str(), "stl")) {
                      assimp_hint = "stl";
                    }
                  }
                  // LOG_DEBUG("import hint " << assimp_hint);
                  auto import_flags =
                      aiProcessPreset_TargetRealtime_MaxQuality |
                      aiProcess_Triangulate;
                  Assimp::Importer ai_importer;
                  ai_importer.SetPropertyFloat(
                      AI_CONFIG_PP_GSN_MAX_SMOOTHING_ANGLE, 60.0);
                  if (import_options.recomputeNormals()) {
                    ai_importer.SetPropertyInteger(
                        AI_CONFIG_PP_RVC_FLAGS,
                        aiComponent_NORMALS |
                            aiComponent_TANGENTS_AND_BITANGENTS);
                    import_flags |= aiProcess_RemoveComponent;
                  }
                  // ai_importer.SetPropertyFloat(
                  //      AI_CONFIG_PP_GSN_MAX_SMOOTHING_ANGLE, 60.0);
                  // ai_importer.SetPropertyBool(AI_CONFIG_FAVOUR_SPEED,
                  // true);
                  /*ai_importer.SetPropertyInteger(
                      AI_CONFIG_PP_RVC_FLAGS,
                      aiComponent_NORMALS |
                          aiComponent_TANGENTS_AND_BITANGENTS);*/
                  auto *ai_scene = ai_importer.ReadFileFromMemory(
                      data.data(), data.size(), import_flags,
                      assimp_hint.c_str());
                  if (ai_scene) {
                    if (!import_options.recomputeNormals() &&
                        import_options.smoothNormals()) {
                      bool has_normapmaps = false;
                      for (size_t i = 0; i < ai_scene->mNumMaterials; i++) {
                        if (ai_scene->mMaterials[i]->GetTextureCount(
                                aiTextureType_NORMALS) > 0) {
                          has_normapmaps = true;
                        }
                      }
                      if (!has_normapmaps) {
                        ai_importer.FreeScene();
                        ai_importer.SetPropertyInteger(
                            AI_CONFIG_PP_RVC_FLAGS,
                            aiComponent_NORMALS |
                                aiComponent_TANGENTS_AND_BITANGENTS);
                        /*ai_scene = ai_importer.ApplyPostProcessing(
                            aiProcessPreset_TargetRealtime_MaxQuality |
                            aiProcess_Triangulate |
                           aiProcess_RemoveComponent);*/
                        import_flags |= aiProcess_RemoveComponent;
                        ai_scene = ai_importer.ReadFileFromMemory(
                            data.data(), data.size(), import_flags,
                            assimp_hint.c_str());
                      }
                    }
                    std::function<void(const Eigen::Isometry3d &,
                                       const aiNode *)>
                        load;
                    load = [&](const Eigen::Isometry3d &parent_pose,
                               const aiNode *node) {
                      // LOG_DEBUG("mesh node " << urdf_mesh->filename << " "
                      //                         << node->mName.C_Str());
                      Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
                      for (size_t row = 0; row < 4; row++) {
                        for (size_t col = 0; col < 4; col++) {
                          pose_matrix(row, col) =
                              node->mTransformation[row][col];
                        }
                      }
                      // LOG_INFO(pose_matrix);
                      // LOG_INFO(Eigen::Isometry3d(pose_matrix).matrix());
                      Eigen::Isometry3d pose =
                          parent_pose * Eigen::Isometry3d(pose_matrix);
                      for (size_t node_mesh_index = 0;
                           node_mesh_index < node->mNumMeshes;
                           node_mesh_index++) {
                        auto *ai_mesh =
                            ai_scene->mMeshes[node->mMeshes[node_mesh_index]];
                        MeshData mesh_data;
                        for (size_t i = 0; i < ai_mesh->mNumVertices; i++) {
                          mesh_data.positions.push_back(Eigen::Vector3f(
                              ai_mesh->mVertices[i].x, ai_mesh->mVertices[i].y,
                              ai_mesh->mVertices[i].z));
                          if (ai_mesh->mNormals) {
                            mesh_data.normals.push_back(Eigen::Vector3f(
                                ai_mesh->mNormals[i].x, ai_mesh->mNormals[i].y,
                                ai_mesh->mNormals[i].z));
                          }
                          if (ai_mesh->mTextureCoords[0]) {
                            mesh_data.texcoords.push_back(Eigen::Vector2f(
                                ai_mesh->mTextureCoords[0][i].x,
                                ai_mesh->mTextureCoords[0][i].y));
                          }
                          if (ai_mesh->mTangents) {
                            mesh_data.tangents.push_back(
                                Eigen::Vector3f(ai_mesh->mTangents[i].x,
                                                ai_mesh->mTangents[i].y,
                                                ai_mesh->mTangents[i].z));
                          }
                          if (ai_mesh->mBitangents) {
                            mesh_data.bitangents.push_back(
                                Eigen::Vector3f(ai_mesh->mBitangents[i].x,
                                                ai_mesh->mBitangents[i].y,
                                                ai_mesh->mBitangents[i].z));
                          }
                        }
                        for (size_t i = 0; i < ai_mesh->mNumFaces; i++) {
                          if (ai_mesh->mFaces[i].mNumIndices == 3) {
                            for (size_t j = 0; j < 3; j++) {
                              mesh_data.indices.push_back(
                                  ai_mesh->mFaces[i].mIndices[j]);
                            }
                          } else if (ai_mesh->mFaces[i].mNumIndices > 3) {
                            LOG_ERROR("invalid triangle "
                                      << ai_mesh->mFaces[i].mNumIndices);
                          }
                        }
                        // mesh_data.computeNormals();
                        if (mesh_data.indices.empty() ||
                            mesh_data.positions.empty()) {
                          LOG_WARN("empty mesh " << urdf_mesh->filename << " "
                                                 << node->mName.C_Str());
                        } else {
                          // auto link = std::make_shared<RobotLink>(
                          //      moveit_link, pose, mesh_data);
                          auto link = std::shared_ptr<RobotLink>(
                              new RobotLink(moveit_link, pose,
                                            std::make_shared<Mesh>(mesh_data)));
                          ObjectScope ws;
                          auto *ai_material =
                              ai_scene->mMaterials[ai_mesh->mMaterialIndex];
                          {
                            aiColor4D color;
                            if (aiGetMaterialColor(ai_material,
                                                   AI_MATKEY_COLOR_DIFFUSE,
                                                   &color) == AI_SUCCESS) {
                              link->material->color().r() = color.r;
                              link->material->color().g() = color.g;
                              link->material->color().b() = color.b;
                            }
                          }
                          auto tex = [&](std::string &url, aiTextureType type) {
                            if (ai_material->GetTextureCount(type) > 0) {
                              aiString str;
                              ai_material->GetTexture(type, 0, &str);
                              // TODO: ???
                              if (const char *new_end = std::strrchr(
                                      urdf_mesh->filename.c_str(), '/')) {
                                url =
                                    std::string(
                                        urdf_mesh->filename.c_str(),
                                        new_end - urdf_mesh->filename.c_str()) +
                                    "/" + str.C_Str();
                              } else {
                                url = str.C_Str();
                              }
                            }
                          };
                          tex(link->material->texture(), aiTextureType_DIFFUSE);
                          tex(link->material->normals(), aiTextureType_NORMALS);
                          link->part_index = ++part_index;
                          links.push_back(link);
                        }
                      }
                      for (size_t i = 0; i < node->mNumChildren; i++) {
                        load(pose, node->mChildren[i]);
                      }
                    };
                    ai_scene->mRootNode->mTransformation = aiMatrix4x4();
                    load(visual_pose *
                             Eigen::Isometry3d(Eigen::Scaling(Eigen::Vector3d(
                                 urdf_mesh->scale.x, urdf_mesh->scale.y,
                                 urdf_mesh->scale.z))),
                         ai_scene->mRootNode);
                    // LOG_INFO("mesh successfully loaded "
                    //         << urdf_mesh->filename);
                  } else {
                    LOG_ERROR("failed to decode mesh " << urdf_mesh->filename);
                  }
                }
              }
            } else {
              LOG_ERROR("  failed to create geometry for link "
                        << urdf_link->name);
            }
          }
        }
      } else {
        LOG_ERROR("link not found " << urdf_link->name);
      }
    }
    LOG_SUCCESS("robot model loaded");
  }
};

struct RobotState : SceneNode {
  std::unordered_set<std::string> variable_names;
  std::vector<std::shared_ptr<MeshRenderer>> mesh_renderers;
  std::shared_ptr<RobotModel> robot_model;
  moveit::core::RobotState moveit_state;
  RobotState(const std::shared_ptr<RobotModel> &robot_model,
             std::shared_ptr<MaterialOverride> material_override)
      : robot_model(robot_model), moveit_state(robot_model->moveit_robot) {
    moveit_state.setToDefaultValues();
    {
      ObjectScope ws;
      for (size_t i = 0; i < robot_model->links.size(); i++) {
        mesh_renderers.push_back(create<MeshRenderer>(
            robot_model->links.at(i)->mesh, robot_model->links.at(i)->material,
            material_override));
      }
    }
    for (auto &n : moveit_state.getVariableNames()) {
      variable_names.insert(n);
    }
  }
};

void RobotDisplayBase::renderSync(const RenderSyncContext &context) {
  MeshDisplayBase::renderSync(context);
  bool invalidated = _invalidated.poll();
  if (invalidated || _watcher.changed(description(), importOptions()) ||
      !_robot_model_loader) {
    static auto manager =
        std::make_shared<ResourceManager<Loader<RobotModel>, std::string,
                                         RobotModelImportOptions>>();
    _robot_model_loader = manager->load(description(), importOptions());
    if (invalidated) {
      _robot_model_loader->clear();
    }
    _robot_state = nullptr;
    GlobalEvents::instance()->redraw();
  }
  if (_robot_state) {
    bool double_sided = doubleSided();
    for (auto &r : _robot_state->mesh_renderers) {
      r->options().double_sided = double_sided;
    }
  }
}

void RobotDisplayBase::renderAsync(const RenderAsyncContext &context) {
  MeshDisplayBase::renderAsync(context);
  if (!_robot_state && _robot_model_loader) {
    if (auto robot_model = _robot_model_loader->load()) {
      if (robot_model->moveit_robot) {
        _robot_state =
            node()->create<RobotState>(robot_model, _material_override);
        GlobalEvents::instance()->redraw();
      }
    }
  }
}

class RobotStateTimeSeriesListener : public TimeSeriesListener {
  std::mutex _mutex;
  std::unordered_map<std::string, double> _positions, _temp;
  void push(const sensor_msgs::JointState &message) {
    for (size_t i = 0; i < message.name.size() && i < message.position.size();
         i++) {
      _temp[message.name.at(i)] = message.position.at(i);
    }
  }

public:
  virtual void push(const std::shared_ptr<const Message> &msg, int64_t start,
                    int64_t end) override {
    if (auto message = msg->instantiate<sensor_msgs::JointState>()) {
      push(*message);
    }
    if (auto message = msg->instantiate<moveit_msgs::DisplayRobotState>()) {
      push(message->state.joint_state);
    }
  }
  virtual void commit() override {
    std::lock_guard<std::mutex> lock(_mutex);
    _positions = _temp;
  }
  void apply(RobotState &robot_state) {
    std::lock_guard<std::mutex> lock(_mutex);
    for (auto &pair : _positions) {
      if (robot_state.variable_names.find(pair.first) !=
          robot_state.variable_names.end()) {
        robot_state.moveit_state.setVariablePosition(pair.first, pair.second);
      }
    }
  }
};

void RobotStateDisplayBase::refreshTopic(const std::string &topic) {
  if (!_subscriber || topic != _subscriber->topic()) {
    _subscriber = std::make_shared<TimeSeriesSubscriber>(topic);
    _subscriber->duration(0.5);
    _listener = std::make_shared<RobotStateTimeSeriesListener>();
    _subscriber->addListener(_listener);
  }
}

void RobotStateDisplayBase::renderSync(const RenderSyncContext &context) {
  if (_robot_state && _listener) {
    _listener->apply(*_robot_state);
    for (size_t i = 0; i < _robot_state->robot_model->links.size(); i++) {
      auto &link = _robot_state->robot_model->links[i];
      _robot_state->mesh_renderers.at(i)->pose(
          pose_temp *
          Eigen::Isometry3d(
              _robot_state->moveit_state
                  .getGlobalLinkTransform(
                      _robot_state->robot_model->moveit_robot->getLinkModel(
                          link->link_index))
                  .matrix()) *
          link->pose);
      _robot_state->mesh_renderers.at(i)->show();
    }
  }
  RobotDisplayBase::renderSync(context);
}

void DisplayRobotStateDisplay::renderSync(const RenderSyncContext &context) {
  // _display_robot_state_message = topic().message();
  auto message = topic().message();
  if (_robot_state && message) {
    try {
      moveit::core::robotStateMsgToRobotState(message->state,
                                              _robot_state->moveit_state);
    } catch (const std::exception &ex) {
      LOG_ERROR(ex.what());
    }
    for (size_t i = 0; i < _robot_state->robot_model->links.size(); i++) {
      auto &link = _robot_state->robot_model->links[i];
      _robot_state->mesh_renderers.at(i)->pose(
          pose_temp *
          Eigen::Isometry3d(
              _robot_state->moveit_state
                  .getGlobalLinkTransform(
                      _robot_state->robot_model->moveit_robot->getLinkModel(
                          link->link_index))
                  .matrix()) *
          link->pose);
      _robot_state->mesh_renderers.at(i)->show();
    }
  }
  RobotDisplayBase::renderSync(context);
}

void DisplayRobotStateDisplay::renderAsync(const RenderAsyncContext &context) {
  RobotDisplayBase::renderAsync(context);
}

/*
void RobotTrajectoryDisplay::renderSync(const RenderSyncContext &context) {

  auto display_trajectory_message = topic().message();
  if (_trajectory_watcher.changed(display_trajectory_message)) {
    _frame_index = 0;
  }

  size_t frame_index = 0;
  if (_robot_state && display_trajectory_message) {
    for (auto &robot_trajectory : display_trajectory_message->trajectory) {
      auto &joint_trajectory = robot_trajectory.joint_trajectory;
      for (auto &point : joint_trajectory.points) {
        if (_frame_index == frame_index) {
          for (size_t ijoint = 0; ijoint < joint_trajectory.joint_names.size();
               ijoint++) {
            if (ijoint < point.positions.size()) {
              _robot_state->moveit_state.setVariablePosition(
                  joint_trajectory.joint_names[ijoint],
                  point.positions[ijoint]);
            }
          }
        }
        frame_index++;
      }
    }
  }
  _frame_index++;
  if (_frame_index >= frame_index) {
    _frame_index = 0;
  }

  if (_robot_state) {
    for (size_t i = 0; i < _robot_state->robot_model->links.size(); i++) {
      auto &link = _robot_state->robot_model->links[i];
      _robot_state->mesh_renderers.at(i)->pose(
          pose_temp *
          Eigen::Isometry3d(
              _robot_state->moveit_state
                  .getGlobalLinkTransform(
                      _robot_state->robot_model->moveit_robot->getLinkModel(
                          link->link_index))
                  .matrix()) *
          link->pose);
      _robot_state->mesh_renderers.at(i)->show();
    }
  }

  RobotDisplayBase::renderSync(context);
}
*/

void RobotTrajectoryDisplay::renderSync(const RenderSyncContext &context) {
  _display_trajectory_message = topic().message();
  _max_steps = maxSteps();
  _show_all = showAllStates();
  if (_update_parameter_watcher.changed(showAllStates(), speed())) {
    std::unique_lock<std::mutex> lock(_update_mutex);
    _update_show_all = showAllStates();
    _update_speed = speed();
    _update_condition.notify_all();
  }
  RobotDisplayBase::renderSync(context);
}

void RobotTrajectoryDisplay::renderAsync(const RenderAsyncContext &context) {
  std::shared_ptr<RobotModel> robot_model;
  if (_robot_model_loader) {
    robot_model = _robot_model_loader->load();
  }
  /*
  size_t current_frame = 0;
  if (!_show_all) {
    current_frame =
        (size_t)std::fmod(ros::WallTime::now().toSec(), _frame_time);
  }
  */
  if (_trajectory_watcher.changed(_display_trajectory_message, _max_steps,
                                  robot_model)) {
    _trajectory.clear();
    if (robot_model && _display_trajectory_message) {
      // LOG_DEBUG("new trajectory");
      if (robot_model->moveit_robot) {

        /*
        ros::Time current_frame_time;
        {
          {
            size_t joint_frame_count = 0;
            size_t md_frame_count = 0;
            for (auto &robot_trajectory :
                 _display_trajectory_message->trajectory) {
              joint_frame_count +=
                  robot_trajectory.joint_trajectory.points.size();
              md_frame_count +=
                  robot_trajectory.multi_dof_joint_trajectory.points.size();
            }
            size_t frame_count = std::max(
                size_t(1), std::max(md_frame_count, joint_frame_count));
            current_frame = current_frame % frame_count;
          }
        }
        */

        /*
        ros::Duration display_time = ros::Duration(0);
        if (!_show_all) {
          ros::Duration trajectory_duration = ros::Duration(0.0);
          {
            for (auto &robot_trajectory :
                 _display_trajectory_message->trajectory) {
              ros::Duration duration = ros::Duration(0.0);
              if (!robot_trajectory.joint_trajectory.points.empty()) {
                duration = std::max(
                    duration, robot_trajectory.joint_trajectory.points.back()
                                  .time_from_start);
              }
              if (!robot_trajectory.multi_dof_joint_trajectory.points.empty()) {
                duration = std::max(
                    duration,
                    robot_trajectory.multi_dof_joint_trajectory.points.back()
                        .time_from_start);
              }
              trajectory_duration += duration;
            }
          }
          display_time = ros::Duration(
              std::fmod(ros::Time::now().toSec(),
                        std::max(0.001, trajectory_duration.toSec())));
        }
        */

        std::map<ros::Duration, std::shared_ptr<RobotState>> states;
        {
          ros::Duration start_time = ros::Duration(0.0);

          for (auto &robot_trajectory :
               _display_trajectory_message->trajectory) {

            {
              auto &joint_trajectory = robot_trajectory.joint_trajectory;
              for (size_t iframe = 0; iframe < joint_trajectory.points.size();
                   iframe++) {
                auto &point = joint_trajectory.points[iframe];
                auto &robot_state = states[start_time + point.time_from_start];
                if (!robot_state) {
                  robot_state = node()->create<RobotState>(robot_model,
                                                           _material_override);
                  robot_state->moveit_state.setToDefaultValues();
                  try {
                    moveit::core::robotStateMsgToRobotState(
                        _display_trajectory_message->trajectory_start,
                        robot_state->moveit_state);
                  } catch (const std::exception &ex) {
                    LOG_ERROR(ex.what());
                  }
                }
                for (size_t ijoint = 0;
                     ijoint <
                     robot_trajectory.joint_trajectory.joint_names.size();
                     ijoint++) {
                  if (ijoint < point.positions.size()) {
                    if (robot_state->variable_names.find(
                            robot_trajectory.joint_trajectory
                                .joint_names[ijoint]) !=
                        robot_state->variable_names.end()) {
                      robot_state->moveit_state.setVariablePosition(
                          robot_trajectory.joint_trajectory.joint_names[ijoint],
                          point.positions[ijoint]);
                    }
                  }
                }
              }
            }

            {
              auto &md_trajectory = robot_trajectory.multi_dof_joint_trajectory;
              for (size_t iframe = 0; iframe < md_trajectory.points.size();
                   iframe++) {
                for (size_t ijoint = 0;
                     ijoint < md_trajectory.joint_names.size(); ijoint++) {
                  auto &joint_name = md_trajectory.joint_names[ijoint];
                  auto &transform =
                      md_trajectory.points[iframe].transforms[ijoint];
                  auto &robot_state =
                      states[start_time +
                             md_trajectory.points[iframe].time_from_start];
                  if (!robot_state) {
                    robot_state = node()->create<RobotState>(
                        robot_model, _material_override);
                    robot_state->moveit_state.setToDefaultValues();
                    try {
                      moveit::core::robotStateMsgToRobotState(
                          _display_trajectory_message->trajectory_start,
                          robot_state->moveit_state);
                    } catch (const std::exception &ex) {
                      LOG_ERROR(ex.what());
                    }
                  }
                  if (auto *joint_model =
                          robot_state->moveit_state.getJointModel(joint_name)) {
                    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
                    tf::transformMsgToEigen(transform, pose);
                    robot_state->moveit_state.setJointPositions(joint_model,
                                                                pose);
                  }
                }
              }
            }

            {
              ros::Duration duration = ros::Duration(0.0);
              if (!robot_trajectory.joint_trajectory.points.empty()) {
                duration = std::max(
                    duration, robot_trajectory.joint_trajectory.points.back()
                                  .time_from_start);
              }
              if (!robot_trajectory.multi_dof_joint_trajectory.points.empty()) {
                duration = std::max(
                    duration,
                    robot_trajectory.multi_dof_joint_trajectory.points.back()
                        .time_from_start);
              }
              start_time += duration;
            }
          }
        }

        _trajectory.clear();
        for (auto &pair : states) {
          auto &robot_state = pair.second;
          robot_state->moveit_state.update();
          for (size_t i = 0; i < robot_state->robot_model->links.size(); i++) {
            auto &link = robot_state->robot_model->links[i];
            robot_state->mesh_renderers.at(i)->pose(
                pose_temp *
                Eigen::Isometry3d(robot_state->moveit_state
                                      .getGlobalLinkTransform(
                                          robot_state->robot_model->moveit_robot
                                              ->getLinkModel(link->link_index))
                                      .matrix()) *
                link->pose);
            // robot_state->mesh_renderers.at(i)->show();
          }
          _trajectory.emplace_back((pair.first - states.begin()->first).toSec(),
                                   robot_state);
        }
      } else {
        LOG_WARN("no robot model");
      }
    }
    GlobalEvents::instance()->redraw();
    {
      std::unique_lock<std::mutex> lock(_update_mutex);
      _update_times.clear();
      for (ssize_t i = 0; i < _trajectory.size(); i++) {
        _update_times.push_back(_trajectory[i].first -
                                _trajectory[std::max(ssize_t(0), i - 1)].first);
      }
      _update_condition.notify_all();
    }
  }

  MeshDisplayBase::renderAsync(context);

  if (!_trajectory.empty()) {
    size_t j = 0;
    {
      std::unique_lock<std::mutex> lock(_update_mutex);
      j = _frame_update;
    }
    j = (j % _trajectory.size());
    for (size_t i = 0; i < _trajectory.size(); i++) {
      auto &pair = _trajectory[i];
      bool vis = (_show_all || i == j);
      for (auto &renderer : pair.second->mesh_renderers) {
        renderer->visible(vis);
      }
    }
  }
}

RobotTrajectoryDisplay::RobotTrajectoryDisplay() {
  _update_thread = std::thread([this]() {
    LOG_DEBUG("start robot trajectory update thread");
    size_t current_frame_index = 0;
    std::chrono::steady_clock::time_point last_tick =
        std::chrono::steady_clock::now();
    std::unique_lock<std::mutex> lock(_update_mutex);
    while (true) {
      if (_update_exit) {
        break;
      }
      if (_update_times.size() < 2 || _update_show_all || _update_speed == 0) {
        _update_condition.wait(lock);
      } else {
        current_frame_index = (current_frame_index % _update_times.size());
        double update_time = _update_times[current_frame_index];
        auto timeout =
            (last_tick + std::chrono::duration<double>(
                             update_time / std::max(1e-12, _update_speed)));
        if (std::chrono::steady_clock::now() >= timeout) {
          current_frame_index++;
          current_frame_index = (current_frame_index % _update_times.size());
          _frame_update = current_frame_index;
          last_tick = std::chrono::steady_clock::now();
          GlobalEvents::instance()->redraw();
          // LOG_DEBUG("redraw robot trajectory display");
        } else {
          _update_condition.wait_until(lock, timeout);
        }
      }
    }
    LOG_DEBUG("exit robot trajectory update thread");
  });
}

RobotTrajectoryDisplay::~RobotTrajectoryDisplay() {
  {
    std::unique_lock<std::mutex> lock(_update_mutex);
    _update_exit = true;
    _update_condition.notify_all();
  }
  _update_thread.join();
}

void RobotModelDisplay::renderSync(const RenderSyncContext &c) {
  auto context = c;
  context.pose.setIdentity();
  _transformer = LockScope()->document()->display()->transformer;
  if (_robot_state) {
    for (size_t i = 0; i < _robot_state->robot_model->links.size(); i++) {
      auto &link = _robot_state->robot_model->links.at(i);
      if (auto transform = link->frame.pose(_transformer)) {
        _robot_state->mesh_renderers.at(i)->pose((*transform) * link->pose);
        _robot_state->mesh_renderers.at(i)->show();
      } else {
        _robot_state->mesh_renderers.at(i)->hide();
      }
    }
  }
  RobotDisplayBase::renderSync(context);
}

void RobotModelDisplay::renderAsync(const RenderAsyncContext &context) {
  RobotDisplayBase::renderAsync(context);
}
