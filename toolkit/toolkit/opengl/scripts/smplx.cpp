#include "toolkit/opengl/scripts/smplx.hpp"
#include "toolkit/anim/scripts/vis.hpp"
#include "toolkit/opengl/components/materials/all.hpp"
#include "toolkit/opengl/editor.hpp"

namespace toolkit::opengl {

static const std::vector<std::string> model_type_names = {"smpl", "smplx"};
static const std::vector<std::string> gender_type_names = {"NEUTRAL", "MALE",
                                                           "FEMALE"};
static const std::vector<std::string> SMPL_JOINT_NAMES = {
    "pelvis",        "left_hip",       "right_hip",    "spine1",
    "left_knee",     "right_knee",     "spine2",       "left_ankle",
    "right_ankle",   "spine3",         "left_foot",    "right_foot",
    "neck",          "left_collar",    "right_collar", "head",
    "left_shoulder", "right_shoulder", "left_elbow",   "right_elbow",
    "left_wrist",    "right_wrist",    "left_hand",    "right_hand",
};
static const std::vector<int> SMPL_JOINT_PARENTS = {
    -1, 0, 0, 0,  1,  2,  3,  4,  5,  6,  7,  8,
    9,  9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21};

void smplx::start() { model_path = "D:\\0tasks\\smplx_archive\\models"; }
void smplx::destroy() {}

void smplx::setup_smplx_model(cnpy::npz_t &data) {}

template <typename T>
void fill_positions(const std::vector<T> &arr, std::vector<math::vector3> &dst,
                    int num) {
  dst.resize(num, math::vector3::Zero());
  for (int i = 0; i < num; i++) {
    dst[i] << arr[3 * i + 0], arr[3 * i + 1], arr[3 * i + 2];
  }
}

template <typename T>
void setup_vertices(mesh_data &mesh_comp, const std::vector<T> &v_template,
                    int num_vertices) {
  mesh_comp.vertices.resize(num_vertices);
  for (int i = 0; i < num_vertices; i++) {
    mesh_comp.vertices[i].position << v_template[i * 3 + 0],
        v_template[i * 3 + 1], v_template[i * 3 + 2], 1.0f;
  }
}

template <typename T>
void setup_indices(mesh_data &mesh_comp, const std::vector<T> &f,
                   int num_faces) {
  mesh_comp.indices.resize(num_faces * 3);
  for (int i = 0; i < 3 * num_faces; i++) {
    mesh_comp.indices[i] = f[i];
  }
  std::vector<int> accum_count(mesh_comp.vertices.size(), 0);
  for (int i = 0; i < num_faces; i++) {
    int vid0 = f[3 * i + 0], vid1 = f[3 * i + 1], vid2 = f[3 * i + 2];
    math::vector3 e01 =
        (mesh_comp.vertices[vid1].position - mesh_comp.vertices[vid0].position)
            .head<3>();
    math::vector3 e12 =
        (mesh_comp.vertices[vid2].position - mesh_comp.vertices[vid1].position)
            .head<3>();
    math::vector4 face_normal;
    face_normal << (e01.cross(e12)).normalized(), 0.0f;
    mesh_comp.vertices[vid0].normal += face_normal;
    mesh_comp.vertices[vid1].normal += face_normal;
    mesh_comp.vertices[vid2].normal += face_normal;
    accum_count[vid0]++;
    accum_count[vid1]++;
    accum_count[vid2]++;
  }
  for (int i = 0; i < mesh_comp.vertices.size(); i++)
    mesh_comp.vertices[i].normal /= accum_count[i];
}

template <typename T>
void setup_skin_weights(mesh_data &mesh_comp, const std::vector<T> &weights,
                        const int num_vertices, const int num_joints) {
  for (int i = 0; i < num_vertices; i++) {
    // find top 4 weighted joints
    for (int j = 0; j < num_joints; j++) {
      if (weights[i * num_joints + j] == 0.0f)
        continue;
      float min_weight = 1e5f;
      int min_weight_idx = 0;
      for (int k = 0; k < 4; k++) {
        if (mesh_comp.vertices[i].bone_weights[k] < min_weight) {
          min_weight = mesh_comp.vertices[i].bone_weights[k];
          min_weight_idx = k;
        }
      }
      if (min_weight < weights[i * num_joints + j]) {
        mesh_comp.vertices[i].bone_weights[min_weight_idx] =
            weights[i * num_joints + j];
        mesh_comp.vertices[i].bone_indices[min_weight_idx] = j;
      }
    }
  }
  // normalize skin weights
  for (int i = 0; i < mesh_comp.vertices.size(); i++) {
    float weight_sum = 0.0f;
    for (int j = 0; j < 4; j++)
      weight_sum += mesh_comp.vertices[i].bone_weights[j];
    if (weight_sum > 0.0f)
      for (int j = 0; j < 4; j++)
        mesh_comp.vertices[i].bone_weights[j] /= weight_sum;
  }
}

void smplx::setup_smpl_model(cnpy::npz_t &data) {
  auto &entity_trans = registry->get<transform>(entity);
  entity_trans.set_world_pos(math::vector3::Zero());
  entity_trans.force_update_hierarchy();

  auto &mesh_comp = registry->emplace_or_replace<mesh_data>(entity);
  mesh_comp.model_name = model_type;
  mesh_comp.mesh_name = gender_type;
  auto mat_ptr = registry->try_get<basic_material>(entity);
  if (mat_ptr == nullptr)
    registry->emplace<basic_material>(entity);
  auto &bundle_data = registry->emplace_or_replace<skinned_mesh_bundle>(entity);
  bundle_data.mesh_entities.push_back(entity);

  // fill data
  auto &v_template = data["v_template"];
  auto &f = data["f"];
  if (v_template.word_size == 4)
    setup_vertices(mesh_comp, v_template.as_vec<float>(), v_template.shape[0]);
  else if (v_template.word_size == 8)
    setup_vertices(mesh_comp, v_template.as_vec<double>(), v_template.shape[0]);
  if (f.word_size == 4)
    setup_indices(mesh_comp, f.as_vec<std::uint32_t>(), f.shape[0]);
  else if (f.word_size == 8)
    setup_indices(mesh_comp, f.as_vec<std::uint64_t>(), f.shape[0]);

  auto &weights = data["weights"];
  if (weights.word_size == 4)
    setup_skin_weights(mesh_comp, weights.as_vec<float>(), weights.shape[0],
                       weights.shape[1]);
  else if (weights.word_size == 8)
    setup_skin_weights(mesh_comp, weights.as_vec<double>(), weights.shape[0],
                       weights.shape[1]);

  // skinned mesh related
  auto &actor_comp = registry->emplace_or_replace<anim::actor>(entity);
  // TODO: can't use emplace_or_replace on a script, could be a problem with
  // script life-cycle
  auto vis_skel_script = registry->try_get<anim::vis_skeleton>(entity);
  if (vis_skel_script == nullptr)
    registry->emplace<anim::vis_skeleton>(entity);
  actor_comp.joint_active.resize(SMPL_JOINT_NAMES.size(), true);
  auto &J = data["J"];
  if (J.word_size == 4)
    fill_positions(J.as_vec<float>(), joint_rest_world_pos, J.shape[0]);
  else if (J.word_size == 8)
    fill_positions(J.as_vec<double>(), joint_rest_world_pos, J.shape[0]);
  for (int i = 0; i < bone_entities.size(); i++)
    registry->destroy(bone_entities[i]);
  bone_entities.clear();
  for (int i = 0; i < SMPL_JOINT_NAMES.size(); i++) {
    bone_entities.push_back(registry->create());
    auto &bone_trans = registry->emplace<transform>(bone_entities[i]);
    bone_trans.name = SMPL_JOINT_NAMES[i];
    actor_comp.ordered_entities.push_back(bone_entities[i]);
    actor_comp.name_to_entity[SMPL_JOINT_NAMES[i]] = bone_entities[i];
    if (SMPL_JOINT_PARENTS[i] != -1) {
      bone_trans.set_parent(bone_entities[SMPL_JOINT_PARENTS[i]]);
      bone_trans.set_local_pos(joint_rest_world_pos[i] -
                               joint_rest_world_pos[SMPL_JOINT_PARENTS[i]]);
    } else {
      bone_trans.set_parent(entity);
      bone_trans.set_local_pos(joint_rest_world_pos[i]);
    }
  }
  auto &root_trans = registry->get<transform>(bone_entities[0]);
  entity_trans.force_update_hierarchy();
  for (int i = 0; i < bone_entities.size(); i++) {
    auto &bone_trans = registry->get<transform>(bone_entities[i]);
    auto &bone_node = registry->emplace<anim::bone_node>(bone_entities[i]);
    bone_node.name = bone_trans.name;
    bone_node.offset_matrix = bone_trans.matrix().inverse();
  }
  bundle_data.bone_entities = bone_entities;

  mesh_comp.update_buffers(true);
}

void smplx::apply_smpl_betas(std::vector<float> betas) {}
void smplx::apply_smplx_betas(std::vector<float> betas) {}

void smplx::preupdate(iapp *app, float dt) {}

void smplx::draw_gui(iapp *app) {
  ImGui::Text(str_format("model_path: %s", model_path.c_str()).c_str());
  if (ImGui::Button("Setup Model Dir", {-1, 30}))
    open_folder_dialog("Select model directory", model_path);
  gui::combo("Model Type", model_index, model_type_names,
             [&](int current) { model_type = model_type_names[current]; });
  gui::combo("Gender Type", gender_index, gender_type_names,
             [&](int current) { gender_type = gender_type_names[current]; });
  if (ImGui::Button("Load Model", {-1, 30})) {
    std::string model_filepath = join_path(
        model_path, model_type == "smplx" ? "smplx_modified" : model_type,
        str_format("%s_%s.npz", uppper_case(model_type).c_str(),
                   gender_type.c_str()));
    if (std::filesystem::exists(model_filepath)) {
      spdlog::info("Load file from {0}", model_filepath);
      smpl_data = cnpy::npz_load(model_filepath);
      if (model_type == "smplx")
        setup_smplx_model(smpl_data);
      else if (model_type == "smpl")
        setup_smpl_model(smpl_data);
    } else {
      spdlog::error("File {0} doesn't exist", model_filepath);
    }
  }
}

void smplx::draw_to_scene(iapp *app) {
  script_draw_to_scene_proxy(
      app, [&](editor *editor, transform &cam_trans, camera &cam_comp) {});
}

}; // namespace toolkit::opengl