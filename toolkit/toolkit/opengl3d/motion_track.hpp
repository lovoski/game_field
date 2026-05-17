/**
 * Motion track — a discrete sequence of per-joint local-transform keyframes.
 *
 * Scope:
 *   Self-contained representation of an animation clip (joint hierarchy +
 *   rest offsets + keyframes), independent from any scene entity.
 *   Binding to a scene actor is done by joint-name matching, see
 *   `motion_player` (toolkit/opengl3d/components/motion_player.hpp).
 *
 * Conventions:
 *   - Quaternions: wxyz (Eigen default).
 *   - Up axis: +Y.
 *   - Length unit: meters.
 *   - Time: seconds (a track stores its sampling fps for reference only).
 */
#pragma once

#include "toolkit/loaders/bvh.hpp"
#include "toolkit/math.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace toolkit::opengl3d {

// Lightweight summary of an FBX file's animation-relevant contents, populated
// by `scan_fbx`. No keyframe sampling happens during a scan, so it's cheap to
// run for a preview UI.
struct fbx_scan_result {
  std::string filepath;
  bool ok = false;
  std::string error;

  bool has_armature = false;
  int num_bones = 0;
  int num_meshes = 0;

  std::vector<std::string> anim_stack_names;
  // [time_begin, time_end] in seconds for each stack.
  std::vector<std::pair<float, float>> anim_stack_times;
};

bool scan_fbx(const std::string &filepath, fbx_scan_result &out);

// One discrete keyframe: time stamp + every joint's local transform.
struct motion_keyframe {
  float time = 0.0f; // seconds since the start of the track
  std::vector<math::vector3> local_pos;
  std::vector<math::quat> local_rot;
};

struct motion_track {
  std::string name;
  float fps = 30.0f;

  // Source info — populated by importers so the engine can re-import this
  // track from its original file during scene deserialization.
  std::string source_file;        // absolute path of the source file
  std::string source_type;        // "bvh" | "fbx"
  std::string source_stack_name;  // FBX only: anim stack name
  float source_fps = 30.0f;       // FBX only: sampling rate used at import

  std::vector<std::string> joint_names;
  std::vector<int> joint_parents;
  std::vector<math::vector3> joint_offsets;

  std::vector<motion_keyframe> keyframes;

  int num_joints() const { return (int)joint_names.size(); }
  int num_frames() const { return (int)keyframes.size(); }
  float duration() const;

  // Evaluate per-joint local transforms at `time` (lerp pos, slerp rot).
  // Output vectors are resized to num_joints().
  void evaluate(float time, std::vector<math::vector3> &out_pos,
                std::vector<math::quat> &out_rot) const;
};

using motion_track_ptr = std::shared_ptr<motion_track>;

// Build a motion track from a parsed BVH file.
motion_track_ptr motion_track_from_bvh(const assets::bvh_data &data,
                                       const std::string &name);
motion_track_ptr motion_track_from_bvh_file(const std::string &filepath);

// Build motion tracks from an FBX file. Each anim_stack in the file becomes
// one motion_track. The track samples the FBX bone hierarchy at `sample_fps`
// from anim_stack.time_begin to anim_stack.time_end.
//
// When `stack_indices` is non-null, only stacks at those indices (matching the
// order returned by `scan_fbx`) are built — used by the import-preview UI to
// avoid sampling unchecked tracks.
std::vector<motion_track_ptr>
motion_tracks_from_fbx_file(const std::string &filepath,
                            float sample_fps = 30.0f,
                            const std::vector<int> *stack_indices = nullptr);

}; // namespace toolkit::opengl3d
