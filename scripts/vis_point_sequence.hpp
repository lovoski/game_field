#pragma once

#include "toolkit/opengl/editor.hpp"
#include "toolkit/scriptable.hpp"
#include "toolkit/system.hpp"

#include "toolkit/loaders/motion.hpp"
#include "toolkit/opengl/draw.hpp"


#include <cnpy.h>

struct vis_point_sequence : public toolkit::scriptable {
  template <typename T> void fill_positions(cnpy::NpyArray &data) {
    auto data_vec = data.as_vec<T>();
    auto nframes = data.shape[0];
    auto njoints = data.shape[1];
    auto nfeats = data.shape[2];
    positions.clear();
    current_frame = 0;
    positions.resize(nframes);
    if (entities.size() > 0) {
      for (int i = 0; i < njoints; i++)
        registry->destroy(entities[i]);
      entities.clear();
    }
    for (int i = 0; i < njoints; i++) {
      auto ent = registry->create();
      auto &trans = registry->emplace<toolkit::transform>(ent);
      trans.name = std::to_string(i);
      entities.push_back(ent);
    }
    for (int i = 0; i < nframes; i++) {
      positions[i].resize(njoints, toolkit::math::vector3::Zero());
      for (int j = 0; j < njoints; j++) {
        positions[i][j] << data_vec[i * njoints * nfeats + j * nfeats + 0],
            data_vec[i * njoints * nfeats + j * nfeats + 1],
            data_vec[i * njoints * nfeats + j * nfeats + 2];
      }
    }
  }

  void start() override {
    parents = {-1, 0, 0, 0,  1,  2,  3,  4,  5,  6,  7,  8,
               9,  9, 9, 12, 13, 14, 16, 17, 18, 19, 20, 21};
  }

  void draw_gui(toolkit::iapp *app) override {
    if (ImGui::Button("Load .npy file", {-1, 30})) {
      std::string filepath;
      if (toolkit::open_file_dialog("Select .npy file", {"*.npy"}, "*.npy",
                                    filepath)) {
        auto data = cnpy::npy_load(filepath);
        if (data.word_size == 4)
          fill_positions<float>(data);
        else if (data.word_size == 8)
          fill_positions<double>(data);
      }
    }
    if (ImGui::Button("Load .bvh file", {-1, 30})) {
      std::string filepath;
      if (toolkit::open_file_dialog("Select .bvh file", {"*.bvh"}, "*.bvh",
                                    filepath)) {
        motion_data.load_from_bvh(filepath, 0.01f);
      }
    }
    ImGui::Checkbox("Auto Play", &auto_play);
    ImGui::DragInt("Current Frame", &current_frame, 1.0f, 0, 100000);
    toolkit::gui::color_edit_3("Positions Color", positions_color);
    toolkit::gui::color_edit_3("Motion Color", motion_color);
  }

  void draw_to_scene(toolkit::iapp *app) override {
    toolkit::opengl::script_draw_to_scene_proxy(
        app, [&](toolkit::opengl::editor *editor, toolkit::transform &cam_trans,
                 toolkit::opengl::camera &cam_comp) {
          if (positions.size() > 0) {
            if (current_frame >= positions.size() || current_frame < 0)
              spdlog::error("Current frame out of range");
            else {
              auto tmp_pos = positions[current_frame];
              toolkit::math::vector4 tmp_vec;
              for (int i = 0; i < tmp_pos.size(); i++) {
                tmp_vec << tmp_pos[i], 1.0;
                tmp_pos[i] =
                    (registry->get<toolkit::transform>(entity).matrix() *
                     tmp_vec)
                        .head<3>();
              }
              std::vector<std::pair<toolkit::math::vector3, toolkit::math::vector3>> bones;
              for (int i = 0; i < parents.size(); i++)
                if (parents[i] != -1)
                  bones.push_back(std::make_pair(tmp_pos[parents[i]], tmp_pos[i]));
              toolkit::opengl::draw_bones(bones, cam_comp.vp, positions_color);
              if (motion_data.skeleton.get_num_joints() != 0) {
                auto frame_data = motion_data.at(current_frame);
                auto motion_pos = frame_data.fk();
                bones.clear();
                toolkit::math::vector4 tmp_vec0, tmp_vec1;
                for (int i = 0; i < motion_data.skeleton.joint_parent.size(); i++) {
                  if (motion_data.skeleton.joint_parent[i] != -1) {
                    tmp_vec0<<motion_pos[motion_data.skeleton.joint_parent[i]],1.0f;
                    tmp_vec1<<motion_pos[i],1.0f;
                    tmp_vec0 = registry->get<toolkit::transform>(entity).matrix()*tmp_vec0;
                    tmp_vec1 = registry->get<toolkit::transform>(entity).matrix()*tmp_vec1;
                    bones.push_back(std::make_pair(tmp_vec0.head<3>(), tmp_vec1.head<3>()));
                  }
                }
                toolkit::opengl::draw_bones(bones, cam_comp.vp, motion_color);
              }
              // toolkit::opengl::draw_wire_spheres(tmp_pos, cam_comp.vp, 0.05f,
              //                                    color);
              for (int i = 0; i < tmp_pos.size(); i++) {
                auto &trans = registry->get<toolkit::transform>(entities[i]);
                trans.set_world_pos(tmp_pos[i]);
              }
            }
          }
        });
  }

  void update(toolkit::iapp* app, float dt) override {
    if (auto_play) {
      current_time += dt;
      current_frame = std::round(current_time * fps);
    }
  }

  int current_frame = 0;
  int fps = 30;
  float current_time = 0.0f;
  bool auto_play = false, motion_loaded = false;
  toolkit::math::vector3 positions_color = toolkit::opengl::Red, motion_color = toolkit::opengl::Purple;
  std::vector<entt::entity> entities;
  std::vector<std::vector<toolkit::math::vector3>> positions;
  std::vector<int> parents;

  toolkit::assets::motion motion_data;
};
DECLARE_SCRIPT(vis_point_sequence, debug)
