/**
 * motion_player — armature-scoped playback component.
 *
 * Attached to an entity that already carries an `actor` component (i.e. an
 * armature in the scene). Holds:
 *   - a list of *bindings* (shared_ptr) to motion tracks owned by the
 *     engine's `motion_library`, and
 *   - the playback state of the currently-active binding.
 *
 * Tracks are not owned here; the same shared_ptr can be bound to multiple
 * armatures' motion_players, so the same clip plays on many rigs without
 * duplication. Apply-time matching is by joint name (see `resolve`); no
 * retargeting is performed today.
 *
 * Engine integration: `engine3d::update_motion_players` ticks every
 * `actor + motion_player` pair each frame. Apps that don't use armatures
 * pay nothing — the system is component-driven.
 *
 * Serialization: scalar playback state (time, speed, loop…) is persisted via
 * REFLECT. Bindings are reconstructed on load by `engine3d::late_deserialize`:
 * it first rebuilds `motion_library` from the source files recorded in each
 * track, then looks up the bound track names (emitted by `late_serialize`)
 * and calls `add_track` to restore the binding list.
 */
#pragma once

#include "toolkit/opengl3d/motion_track.hpp"
#include "toolkit/system.hpp"

namespace toolkit::opengl3d {

struct motion_player : public icomponent {
  // Bindings to library tracks. The component does not own the tracks; the
  // engine's motion_library does.
  std::vector<motion_track_ptr> tracks;

  // Index into `tracks`. -1 means "no active binding".
  int active_track = -1;

  float time = 0.0f;
  float speed = 1.0f;
  bool playing = false;
  bool loop = true;
  bool apply_root_position = true;

  // Resolved cache: size == tracks[active_track]->num_joints(), each entry is
  // an index into the actor's `ordered_entities` (-1 = no match).
  std::vector<int> track_to_actor;

  // Rebuild `track_to_actor` for the current active track + actor.
  void resolve(entt::registry &registry, entt::entity self);

  // Evaluate the active track at `time` and push the local transforms onto
  // the actor's joint entities.
  void apply_pose(entt::registry &registry, entt::entity self);

  // Advance `time` by `dt * speed`, honoring loop/clamp.
  void tick(float dt);

  motion_track_ptr active() const {
    if (active_track < 0 || active_track >= (int)tracks.size())
      return nullptr;
    return tracks[active_track];
  }

  // Switch the active track and re-resolve the joint map. Resets time to 0.
  void set_active(entt::registry &registry, entt::entity self, int idx);
  void add_track(motion_track_ptr track);

  void draw_gui(entt::registry &registry, entt::entity entity) override;

  // Serializes the names of bound tracks so the engine can rebind them after
  // scene reload once the motion_library is reconstructed. The actual rebind
  // happens in engine3d::late_deserialize, which runs after the library is
  // rebuilt and after all component late_deserializes complete.
  nlohmann::json late_serialize(entt::registry &registry,
                                entt::entity entity) override;
};

// Scalar playback state only — see header comment.
DECLARE_COMPONENT(motion_player, animation, active_track, time, speed, playing,
                  loop, apply_root_position)

}; // namespace toolkit::opengl3d
