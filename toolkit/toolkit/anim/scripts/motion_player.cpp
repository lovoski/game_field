#include "toolkit/anim/scripts/motion_player.hpp"
#include "toolkit/anim/scripts/vis.hpp"

namespace toolkit::anim {

void bvh_motion_player::update(entt::registry &registry, float dt) {
  int active_script_count = registry.view<bvh_motion_player>().size();
  if (auto_play)
    current_time += dt * play_speed / active_script_count;
}

void bvh_motion_player::lateupdate(entt::registry &registry, float dt) {
  auto actor_comp = registry.try_get<anim::actor>(entity);
  if ((actor_comp != nullptr) && motion_loaded && apply_motion) {
    int start_frame = current_time / motion.frametime;
    int end_frame = start_frame + 1;
    float blending_alpha =
        std::clamp(current_time / motion.frametime - start_frame, 0.0f, 1.0f);
    for (int i = 0; i < motion.names.size(); i++) {
      auto bvh_joint_name = motion.names[i];
      if (actor_comp->name_to_entity.count(bvh_joint_name) != 0) {
        auto &joint_trans = registry.get<transform>(
            actor_comp->name_to_entity[bvh_joint_name]);
        if (current_time < 0) {
          joint_trans.set_local_pos(motion.local_pos[0][i]);
          joint_trans.set_local_rot(motion.local_rot[0][i]);
        } else {
          if (start_frame >= motion.local_pos.size() - 1) {
            joint_trans.set_local_pos(
                motion.local_pos[motion.local_pos.size() - 1][i]);
            joint_trans.set_local_rot(
                motion.local_rot[motion.local_pos.size() - 1][i]);
          } else {
            joint_trans.set_local_pos(
                motion.local_pos[start_frame][i] * (1.0f - blending_alpha) +
                motion.local_pos[end_frame][i] * blending_alpha);
            joint_trans.set_local_rot(motion.local_rot[start_frame][i].slerp(
                blending_alpha, motion.local_rot[end_frame][i]));
          }
        }
      }
    }
  }
}

entt::entity bvh_motion_player::load_motion(entt::registry &registry, std::string filepath) {
  motion = assets::load_bvh(filepath);
  create_bvh_actor(registry, motion, entity);
  auto &container_trans = registry.get<transform>(entity);
  auto &vis_script = registry.get<vis_skeleton>(entity);
  vis_script.bone_color =
      math::vector3(math::rand(0, 1), math::rand(0, 1), math::rand(0, 1));
  container_trans.name =
      str_format("entity: %d (%s)", entt::to_integral(entity),
                 std::filesystem::path(filepath).filename().string().c_str());
  motion_loaded = true;
  return entity;
}

void bvh_motion_player::draw_gui(entt::registry &registry, entt::entity entity) {
  ImGui::SeparatorText("Shared Variables");
  ImGui::Checkbox("Auto Play", &auto_play);
  ImGui::DragFloat("Play Speed", &play_speed, 0.001f, -100.0f, 100.0f);
  ImGui::DragFloat("Time (Seconds)", &current_time, 0.001f, 0.0f, FLT_MAX);

  ImGui::SeparatorText("Motion Settings");
  ImGui::Checkbox("Apply Motion", &apply_motion);
  ImGui::Text("Motion Duration: %.3f s, played %.2f %%",
              motion_loaded ? motion.local_pos.size() * motion.frametime : 0.0f,
              motion_loaded
                  ? std::clamp(current_time /
                                   (motion.local_pos.size() * motion.frametime),
                               0.0f, 1.0f) *
                        100.0f
                  : 0.0f);
  if (ImGui::Button("Import Motion", {-1, 30})) {
    std::string filepath;
    if (open_file_dialog("Slect Motion File", {"*.bvh"}, filepath)) {
      load_motion(registry, filepath);
    }
  }
}

void import_all_bvh_motion(entt::registry &registry, std::string dirpath,
                           float scale) {
  listdir(dirpath, [&](std::string filepath) {
    if (!endswith(filepath, ".bvh"))
      return;
    spdlog::info("Load .bvh file from {0}", filepath);
    auto container = registry.create();
    auto &container_trans = registry.emplace<transform>(container);
    container_trans.name = std::filesystem::path(filepath).string();
    auto &motion_player = registry.emplace<bvh_motion_player>(container);
    motion_player.load_motion(registry, filepath);
    container_trans.set_world_scale(math::vector3(scale, scale, scale));
  });
}

}; // namespace toolkit::anim