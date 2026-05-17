#include "toolkit/opengl3d/motion_track.hpp"

#include "toolkit/utils.hpp"

#include <SDL.h>
#include <algorithm>
#include <filesystem>
#include <map>
#include <stack>

#include <ufbx.h>

namespace toolkit::opengl3d {

float motion_track::duration() const {
  if (keyframes.empty())
    return 0.0f;
  return keyframes.back().time - keyframes.front().time;
}

void motion_track::evaluate(float time, std::vector<math::vector3> &out_pos,
                            std::vector<math::quat> &out_rot) const {
  int n = num_joints();
  out_pos.assign(n, math::vector3::Zero());
  out_rot.assign(n, math::quat::Identity());
  if (keyframes.empty())
    return;

  if (time <= keyframes.front().time) {
    out_pos = keyframes.front().local_pos;
    out_rot = keyframes.front().local_rot;
    return;
  }
  if (time >= keyframes.back().time) {
    out_pos = keyframes.back().local_pos;
    out_rot = keyframes.back().local_rot;
    return;
  }
  auto it = std::upper_bound(
      keyframes.begin(), keyframes.end(), time,
      [](float t, const motion_keyframe &k) { return t < k.time; });
  int i1 = (int)std::distance(keyframes.begin(), it);
  int i0 = std::max(0, i1 - 1);
  const auto &k0 = keyframes[i0];
  const auto &k1 = keyframes[i1];
  float dt = k1.time - k0.time;
  float alpha = dt > 1e-6f ? (time - k0.time) / dt : 0.0f;
  for (int j = 0; j < n; j++) {
    out_pos[j] = k0.local_pos[j] * (1.0f - alpha) + k1.local_pos[j] * alpha;
    out_rot[j] = k0.local_rot[j].slerp(alpha, k1.local_rot[j]);
  }
}

motion_track_ptr motion_track_from_bvh(const assets::bvh_data &data,
                                       const std::string &name) {
  auto track = std::make_shared<motion_track>();
  track->name = name;
  track->fps = data.frametime > 1e-6f ? (1.0f / data.frametime) : 30.0f;
  track->joint_names = data.names;
  track->joint_parents = data.parents;
  track->joint_offsets = data.offsets;

  int njoints = (int)data.names.size();
  int nframes = (int)data.local_rot.size();
  track->keyframes.resize(nframes);
  for (int f = 0; f < nframes; f++) {
    motion_keyframe &kf = track->keyframes[f];
    kf.time = data.frametime * (float)f;
    kf.local_pos.resize(njoints, math::vector3::Zero());
    kf.local_rot.resize(njoints, math::quat::Identity());
    for (int j = 0; j < njoints; j++) {
      kf.local_pos[j] = data.local_pos[f][j];
      kf.local_rot[j] = data.local_rot[f][j];
    }
  }
  return track;
}

motion_track_ptr motion_track_from_bvh_file(const std::string &filepath) {
  auto data = assets::load_bvh(filepath);
  auto name = std::filesystem::path(filepath).filename().string();
  auto t = motion_track_from_bvh(data, name);
  if (t) {
    t->source_file = filepath;
    t->source_type = "bvh";
  }
  return t;
}

namespace {

inline math::vector3 ufbx_to_vec3(ufbx_vec3 v) {
  return math::vector3((float)v.x, (float)v.y, (float)v.z);
}
inline math::quat ufbx_to_quat(ufbx_quat v) {
  return math::quat((float)v.w, (float)v.x, (float)v.y, (float)v.z);
}

ufbx_node *find_first_bone_parent(ufbx_node *node) {
  if (!node)
    return nullptr;
  ufbx_node *p = node->parent;
  while (p) {
    if (p->bone)
      return p;
    p = p->parent;
  }
  return nullptr;
}

std::vector<ufbx_node *> collect_bone_nodes(ufbx_scene *scene) {
  std::vector<ufbx_node *> bones;
  std::stack<ufbx_node *> stk;
  stk.push(scene->root_node);
  while (!stk.empty()) {
    ufbx_node *n = stk.top();
    stk.pop();
    if (n != scene->root_node && n->bone)
      bones.push_back(n);
    for (size_t i = n->children.count; i-- > 0;)
      stk.push(n->children[i]);
  }
  return bones;
}

motion_track_ptr build_track_from_anim(ufbx_scene *scene, ufbx_anim *anim,
                                       double t0, double t1,
                                       const std::vector<ufbx_node *> &bones,
                                       const std::string &name,
                                       float sample_fps) {
  auto track = std::make_shared<motion_track>();
  track->name = name;
  track->fps = sample_fps;

  int njoints = (int)bones.size();
  std::map<ufbx_node *, int> bone_index;
  for (int i = 0; i < njoints; i++)
    bone_index[bones[i]] = i;

  track->joint_names.resize(njoints);
  track->joint_parents.resize(njoints, -1);
  track->joint_offsets.resize(njoints, math::vector3::Zero());
  for (int i = 0; i < njoints; i++) {
    track->joint_names[i] = std::string(bones[i]->name.data);
    auto pbone = find_first_bone_parent(bones[i]);
    track->joint_parents[i] =
        (pbone && bone_index.count(pbone)) ? bone_index[pbone] : -1;
    track->joint_offsets[i] =
        ufbx_to_vec3(bones[i]->local_transform.translation);
  }

  if (t1 <= t0)
    t1 = t0 + 1.0;
  double dt = sample_fps > 1e-6f ? (1.0 / (double)sample_fps) : (1.0 / 30.0);
  int nframes = std::max(1, (int)std::ceil((t1 - t0) / dt) + 1);

  track->keyframes.resize(nframes);
  for (int f = 0; f < nframes; f++) {
    double t = std::min(t1, t0 + dt * (double)f);
    auto &kf = track->keyframes[f];
    kf.time = (float)(t - t0);
    kf.local_pos.resize(njoints, math::vector3::Zero());
    kf.local_rot.resize(njoints, math::quat::Identity());
    for (int j = 0; j < njoints; j++) {
      ufbx_transform xf = ufbx_evaluate_transform(anim, bones[j], t);
      kf.local_pos[j] = ufbx_to_vec3(xf.translation);
      kf.local_rot[j] = ufbx_to_quat(xf.rotation);
    }
  }
  return track;
}

} // anonymous namespace

std::vector<motion_track_ptr>
motion_tracks_from_fbx_file(const std::string &filepath, float sample_fps,
                            const std::vector<int> *stack_indices) {
  std::vector<motion_track_ptr> tracks;

  ufbx_error error;
  ufbx_load_opts opts = {};
  opts.load_external_files = true;
  opts.ignore_missing_external_files = true;
  opts.generate_missing_normals = true;
  opts.target_axes.right = UFBX_COORDINATE_AXIS_POSITIVE_X;
  opts.target_axes.up = UFBX_COORDINATE_AXIS_POSITIVE_Y;
  opts.target_axes.front = UFBX_COORDINATE_AXIS_POSITIVE_Z;
  opts.target_unit_meters = 1.0f;

  ufbx_scene *scene = ufbx_load_file(filepath.c_str(), &opts, &error);
  if (!scene) {
    SDL_Log("motion_tracks_from_fbx_file: failed to load %s: %s",
            filepath.c_str(), error.description.data);
    return tracks;
  }

  auto bones = collect_bone_nodes(scene);
  if (bones.empty()) {
    SDL_Log("motion_tracks_from_fbx_file: %s has no bones", filepath.c_str());
    ufbx_free_scene(scene);
    return tracks;
  }

  auto stack_selected = [&](int i) -> bool {
    if (!stack_indices)
      return true;
    for (int v : *stack_indices)
      if (v == i)
        return true;
    return false;
  };

  std::string base = std::filesystem::path(filepath).filename().string();
  for (size_t i = 0; i < scene->anim_stacks.count; i++) {
    if (!stack_selected((int)i))
      continue;
    ufbx_anim_stack *stack = scene->anim_stacks[i];
    std::string stack_name =
        stack->name.length > 0 ? std::string(stack->name.data) : "anim";
    std::string track_name = base + ":" + stack_name;
    auto track = build_track_from_anim(scene, stack->anim, stack->time_begin,
                                           stack->time_end, bones, track_name,
                                           sample_fps);
    if (track) {
      track->source_file = filepath;
      track->source_type = "fbx";
      track->source_stack_name = stack_name;
      track->source_fps = sample_fps;
    }
    tracks.push_back(track);
  }
  if (tracks.empty() && scene->anim_stacks.count == 0 && scene->anim &&
      (stack_indices == nullptr || stack_indices->empty() ||
       (stack_indices->size() == 1 && stack_indices->front() == 0))) {
    auto track = build_track_from_anim(
        scene, scene->anim, scene->anim->time_begin, scene->anim->time_end,
        bones, base + ":anim", sample_fps);
    if (track) {
      track->source_file = filepath;
      track->source_type = "fbx";
      track->source_stack_name = "anim";
      track->source_fps = sample_fps;
    }
    tracks.push_back(track);
  }

  ufbx_free_scene(scene);
  return tracks;
}

bool scan_fbx(const std::string &filepath, fbx_scan_result &out) {
  out = fbx_scan_result{};
  out.filepath = filepath;

  ufbx_error error;
  ufbx_load_opts opts = {};
  opts.load_external_files = false;
  opts.ignore_missing_external_files = true;
  // Skip geometry to make the scan cheap. We only need node + anim metadata.
  opts.ignore_geometry = true;
  opts.ignore_embedded = true;
  opts.target_axes.right = UFBX_COORDINATE_AXIS_POSITIVE_X;
  opts.target_axes.up = UFBX_COORDINATE_AXIS_POSITIVE_Y;
  opts.target_axes.front = UFBX_COORDINATE_AXIS_POSITIVE_Z;
  opts.target_unit_meters = 1.0f;

  ufbx_scene *scene = ufbx_load_file(filepath.c_str(), &opts, &error);
  if (!scene) {
    out.ok = false;
    out.error = error.description.data ? error.description.data : "unknown";
    return false;
  }

  auto bones = collect_bone_nodes(scene);
  out.num_bones = (int)bones.size();
  out.has_armature = !bones.empty();
  out.num_meshes = (int)scene->meshes.count;

  out.anim_stack_names.reserve(scene->anim_stacks.count);
  out.anim_stack_times.reserve(scene->anim_stacks.count);
  for (size_t i = 0; i < scene->anim_stacks.count; i++) {
    ufbx_anim_stack *stack = scene->anim_stacks[i];
    std::string nm =
        stack->name.length > 0 ? std::string(stack->name.data) : "anim";
    out.anim_stack_names.push_back(nm);
    out.anim_stack_times.emplace_back((float)stack->time_begin,
                                      (float)stack->time_end);
  }

  out.ok = true;
  ufbx_free_scene(scene);
  return true;
}

}; // namespace toolkit::opengl3d
