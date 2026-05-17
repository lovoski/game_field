#include "toolkit/opengl3d/components/motion_player.hpp"

#include "toolkit/opengl3d/components/actor.hpp"

#include <imgui.h>

namespace toolkit::opengl3d {

void motion_player::resolve(entt::registry &registry, entt::entity self) {
  track_to_actor.clear();
  auto track = active();
  if (!track)
    return;
  if (!registry.valid(self) || !registry.all_of<actor>(self))
    return;
  auto &a = registry.get<actor>(self);
  track_to_actor.resize(track->num_joints(), -1);
  for (int j = 0; j < track->num_joints(); j++) {
    auto it = a.name_to_entity.find(track->joint_names[j]);
    if (it == a.name_to_entity.end())
      continue;
    for (int k = 0; k < (int)a.ordered_entities.size(); k++) {
      if (a.ordered_entities[k] == it->second) {
        track_to_actor[j] = k;
        break;
      }
    }
  }
}

void motion_player::apply_pose(entt::registry &registry, entt::entity self) {
  auto track = active();
  if (!track)
    return;
  if (!registry.valid(self) || !registry.all_of<actor>(self))
    return;
  auto &a = registry.get<actor>(self);

  std::vector<math::vector3> pos;
  std::vector<math::quat> rot;
  track->evaluate(time, pos, rot);

  int n = track->num_joints();
  for (int j = 0; j < n; j++) {
    int ai = j < (int)track_to_actor.size() ? track_to_actor[j] : -1;
    if (ai < 0 || ai >= (int)a.ordered_entities.size())
      continue;
    auto ent = a.ordered_entities[ai];
    if (!registry.valid(ent))
      continue;
    auto &tr = registry.get<transform>(ent);
    bool is_root = (track->joint_parents[j] == -1);
    if (is_root && apply_root_position)
      tr.set_local_pos(pos[j]);
    tr.set_local_rot(rot[j]);
  }
}

void motion_player::tick(float dt) {
  auto track = active();
  if (!track || !playing)
    return;
  time += dt * speed;
  float dur = track->duration();
  if (loop && dur > 1e-6f) {
    while (time > dur)
      time -= dur;
    while (time < 0.0f)
      time += dur;
  } else {
    if (time > dur) {
      time = dur;
      playing = false;
    }
    if (time < 0.0f) {
      time = 0.0f;
      playing = false;
    }
  }
}

void motion_player::set_active(entt::registry &registry, entt::entity self,
                               int idx) {
  if (idx < -1 || idx >= (int)tracks.size())
    return;
  active_track = idx;
  time = 0.0f;
  resolve(registry, self);
}

void motion_player::add_track(motion_track_ptr track) {
  if (!track)
    return;
  tracks.push_back(track);
  if (active_track < 0)
    active_track = (int)tracks.size() - 1;
}

nlohmann::json motion_player::late_serialize(entt::registry &registry,
                                             entt::entity entity) {
  nlohmann::json j;
  j["bound_tracks"] = nlohmann::json::array();
  for (auto &t : tracks)
    j["bound_tracks"].push_back(t ? t->name : "");
  j["active_track_name"] = active() ? active()->name : "";
  return j;
}

void motion_player::draw_gui(entt::registry &registry, entt::entity entity) {
  ImGui::Text("Bindings: %d", (int)tracks.size());
  ImGui::SameLine();
  if (ImGui::SmallButton("Unbind All##mp")) {
    tracks.clear();
    active_track = -1;
    track_to_actor.clear();
  }

  // Active track combo.
  const char *cur = "<none>";
  if (auto t = active())
    cur = t->name.c_str();
  ImGui::SetNextItemWidth(-FLT_MIN);
  if (ImGui::BeginCombo("##mp_track", cur)) {
    if (ImGui::Selectable("<none>", active_track < 0))
      set_active(registry, entity, -1);
    for (int k = 0; k < (int)tracks.size(); k++) {
      bool sel = (k == active_track);
      if (ImGui::Selectable(tracks[k]->name.c_str(), sel))
        set_active(registry, entity, k);
    }
    ImGui::EndCombo();
  }

  // Bound-track list with unbind buttons (tracks remain in the engine
  // motion library — this only severs the binding to this armature).
  if (!tracks.empty() && ImGui::TreeNode("Bound Tracks")) {
    int remove_idx = -1;
    for (int i = 0; i < (int)tracks.size(); i++) {
      ImGui::PushID(i);
      ImGui::BulletText("%s", tracks[i]->name.c_str());
      ImGui::SameLine();
      ImGui::TextDisabled("%dj %df %.2fs", tracks[i]->num_joints(),
                          tracks[i]->num_frames(), tracks[i]->duration());
      ImGui::SameLine();
      if (ImGui::SmallButton("X"))
        remove_idx = i;
      ImGui::PopID();
    }
    if (remove_idx >= 0) {
      tracks.erase(tracks.begin() + remove_idx);
      if (active_track == remove_idx) {
        active_track = tracks.empty() ? -1 : 0;
        time = 0.0f;
        resolve(registry, entity);
      } else if (active_track > remove_idx) {
        active_track--;
      }
    }
    ImGui::TreePop();
  }

  // Joint match diagnostic.
  if (auto t = active()) {
    int matched = 0;
    for (int v : track_to_actor)
      if (v >= 0)
        matched++;
    ImGui::TextDisabled("Joint match: %d / %d", matched, t->num_joints());
    ImGui::SameLine();
    if (ImGui::SmallButton("Re-resolve"))
      resolve(registry, entity);
  }

  if (ImGui::Button(playing ? "Pause" : "Play"))
    playing = !playing;
  ImGui::SameLine();
  if (ImGui::Button("Stop")) {
    playing = false;
    time = 0.0f;
    apply_pose(registry, entity);
  }

  float dur = 0.0f;
  if (auto t = active())
    dur = t->duration();
  ImGui::SetNextItemWidth(-FLT_MIN);
  if (ImGui::SliderFloat("##mp_time", &time, 0.0f, std::max(dur, 1e-3f),
                         "%.3fs"))
    apply_pose(registry, entity);

  ImGui::SetNextItemWidth(120);
  ImGui::DragFloat("Speed", &speed, 0.01f, -10.0f, 10.0f);
  ImGui::SameLine();
  ImGui::Checkbox("Loop", &loop);
  ImGui::SameLine();
  ImGui::Checkbox("Root pos", &apply_root_position);
}

}; // namespace toolkit::opengl3d
